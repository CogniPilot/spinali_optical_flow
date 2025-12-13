"""
ROS 2 node that subscribes to ArgusResults and publishes a 4x8 PointCloud2.
Optionally fuses IMU and magnetometer data using Madgwick filter for orientation.

Usage:
    ros2 run spinali_optical_flow_integration argus_to_pointcloud
"""

import rclpy
from rclpy.node import Node
from afbr_msgs.msg import ArgusResults
from sensor_msgs.msg import PointCloud2, PointField, Imu, MagneticField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import struct
import math
import numpy as np

# Pixel grid dimensions (Y x X)
GRID_HEIGHT = 4
GRID_WIDTH = 8
TOTAL_PIXELS = GRID_HEIGHT * GRID_WIDTH  # 32 pixels

# Pixel pitch in meters (X-Y position when Z=0)
PIXEL_PITCH_X = 0.0005  # .0005m between pixels in X
PIXEL_PITCH_Y = 0.0005  # .0005m between pixels in Y

# Field of view angles (degrees per pixel)
FOV_HORIZONTAL = 2.0  # degrees per pixel in X (horizontal)
FOV_VERTICAL = 2.0    # degrees per pixel in Y (vertical)


class MadgwickFilter:
    """Simple Madgwick AHRS filter implementation."""

    def __init__(self, beta=0.1):
        self.beta = beta  # Filter gain
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion [w, x, y, z]
        self.last_time = None

    def update(self, gyro, accel, mag, timestamp):
        """
        Update orientation estimate with IMU and magnetometer data.
        gyro: [gx, gy, gz] in rad/s
        accel: [ax, ay, az] in m/s^2
        mag: [mx, my, mz] in Tesla (or None to use IMU-only mode)
        timestamp: time in seconds
        """
        if self.last_time is None:
            self.last_time = timestamp
            return self.q.copy()

        dt = timestamp - self.last_time
        self.last_time = timestamp

        if dt <= 0 or dt > 1.0:
            return self.q.copy()

        q = self.q
        gx, gy, gz = gyro
        ax, ay, az = accel

        # Normalize accelerometer
        accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if accel_norm < 1e-10:
            return self.q.copy()
        ax, ay, az = ax/accel_norm, ay/accel_norm, az/accel_norm

        q0, q1, q2, q3 = q[0], q[1], q[2], q[3]

        if mag is not None and any(m != 0 for m in mag):
            mx, my, mz = mag
            # Normalize magnetometer
            mag_norm = math.sqrt(mx*mx + my*my + mz*mz)
            if mag_norm > 1e-10:
                mx, my, mz = mx/mag_norm, my/mag_norm, mz/mag_norm

                # Reference direction of Earth's magnetic field
                hx = 2*mx*(0.5 - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2)
                hy = 2*mx*(q1*q2 + q0*q3) + 2*my*(0.5 - q1*q1 - q3*q3) + 2*mz*(q2*q3 - q0*q1)
                bx = math.sqrt(hx*hx + hy*hy)
                bz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + 2*mz*(0.5 - q1*q1 - q2*q2)

                # Gradient descent corrective step (with magnetometer)
                s0 = -2*q2*(2*q1*q3 - 2*q0*q2 - ax) + 2*q1*(2*q0*q1 + 2*q2*q3 - ay) - bz*q2*(bx*(0.5 - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx) + (-bx*q3 + bz*q1)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my) + bx*q2*(bx*(q0*q2 + q1*q3) + bz*(0.5 - q1*q1 - q2*q2) - mz)
                s1 = 2*q3*(2*q1*q3 - 2*q0*q2 - ax) + 2*q0*(2*q0*q1 + 2*q2*q3 - ay) - 4*q1*(1 - 2*q1*q1 - 2*q2*q2 - az) + bz*q3*(bx*(0.5 - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx) + (bx*q2 + bz*q0)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my) + (bx*q3 - 2*bz*q1)*(bx*(q0*q2 + q1*q3) + bz*(0.5 - q1*q1 - q2*q2) - mz)
                s2 = -2*q0*(2*q1*q3 - 2*q0*q2 - ax) + 2*q3*(2*q0*q1 + 2*q2*q3 - ay) - 4*q2*(1 - 2*q1*q1 - 2*q2*q2 - az) + (-2*bx*q2 - bz*q0)*(bx*(0.5 - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx) + (bx*q1 + bz*q3)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my) + (bx*q0 - 2*bz*q2)*(bx*(q0*q2 + q1*q3) + bz*(0.5 - q1*q1 - q2*q2) - mz)
                s3 = 2*q1*(2*q1*q3 - 2*q0*q2 - ax) + 2*q2*(2*q0*q1 + 2*q2*q3 - ay) + (-2*bx*q3 + bz*q1)*(bx*(0.5 - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2) - mx) + (-bx*q0 + bz*q2)*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3) - my) + bx*q1*(bx*(q0*q2 + q1*q3) + bz*(0.5 - q1*q1 - q2*q2) - mz)
            else:
                # Fall back to IMU-only
                s0 = -2*q2*(2*q1*q3 - 2*q0*q2 - ax) + 2*q1*(2*q0*q1 + 2*q2*q3 - ay)
                s1 = 2*q3*(2*q1*q3 - 2*q0*q2 - ax) + 2*q0*(2*q0*q1 + 2*q2*q3 - ay) - 4*q1*(1 - 2*q1*q1 - 2*q2*q2 - az)
                s2 = -2*q0*(2*q1*q3 - 2*q0*q2 - ax) + 2*q3*(2*q0*q1 + 2*q2*q3 - ay) - 4*q2*(1 - 2*q1*q1 - 2*q2*q2 - az)
                s3 = 2*q1*(2*q1*q3 - 2*q0*q2 - ax) + 2*q2*(2*q0*q1 + 2*q2*q3 - ay)
        else:
            # IMU-only gradient descent
            s0 = -2*q2*(2*q1*q3 - 2*q0*q2 - ax) + 2*q1*(2*q0*q1 + 2*q2*q3 - ay)
            s1 = 2*q3*(2*q1*q3 - 2*q0*q2 - ax) + 2*q0*(2*q0*q1 + 2*q2*q3 - ay) - 4*q1*(1 - 2*q1*q1 - 2*q2*q2 - az)
            s2 = -2*q0*(2*q1*q3 - 2*q0*q2 - ax) + 2*q3*(2*q0*q1 + 2*q2*q3 - ay) - 4*q2*(1 - 2*q1*q1 - 2*q2*q2 - az)
            s3 = 2*q1*(2*q1*q3 - 2*q0*q2 - ax) + 2*q2*(2*q0*q1 + 2*q2*q3 - ay)

        # Normalize step
        s_norm = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if s_norm > 1e-10:
            s0, s1, s2, s3 = s0/s_norm, s1/s_norm, s2/s_norm, s3/s_norm

        # Rate of change of quaternion from gyroscope
        qDot0 = 0.5 * (-q1*gx - q2*gy - q3*gz) - self.beta * s0
        qDot1 = 0.5 * (q0*gx + q2*gz - q3*gy) - self.beta * s1
        qDot2 = 0.5 * (q0*gy - q1*gz + q3*gx) - self.beta * s2
        qDot3 = 0.5 * (q0*gz + q1*gy - q2*gx) - self.beta * s3

        # Integrate
        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt

        # Normalize quaternion
        q_norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q = np.array([q0/q_norm, q1/q_norm, q2/q_norm, q3/q_norm])

        return self.q.copy()

    def get_quaternion(self):
        """Return current orientation as [w, x, y, z]."""
        return self.q.copy()


class ArgusToPointCloud(Node):
    def __init__(self):
        super().__init__('argus_to_pointcloud')

        # Parameters
        self.declare_parameter('argus_topic', 'out/argus_results')
        self.declare_parameter('output_topic', 'argus_pointcloud')
        self.declare_parameter('pixel_pitch_x', PIXEL_PITCH_X)
        self.declare_parameter('pixel_pitch_y', PIXEL_PITCH_Y)
        self.declare_parameter('fov_horizontal', FOV_HORIZONTAL)
        self.declare_parameter('fov_vertical', FOV_VERTICAL)
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('sensor_frame', 'afbr-s50lx85d')
        self.declare_parameter('tf_child_frame', 'spinali_optical_flow_link')
        self.declare_parameter('publish_tf', True)
        # Transform from parent to tf_child_frame
        self.declare_parameter('tf_x', 0.0)
        self.declare_parameter('tf_y', 0.0)
        self.declare_parameter('tf_z', 0.0)
        self.declare_parameter('tf_roll', 0.0)
        self.declare_parameter('tf_pitch', 0.0)
        self.declare_parameter('tf_yaw', 0.0)
        # IMU/Magnetometer fusion parameters
        self.declare_parameter('use_imu_orientation', False)
        self.declare_parameter('imu_topic', 'out/imu')
        self.declare_parameter('mag_topic', 'out/magnetic_field')
        self.declare_parameter('madgwick_beta', 0.1)

        argus_topic = self.get_parameter('argus_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.pixel_pitch_x = self.get_parameter('pixel_pitch_x').value
        self.pixel_pitch_y = self.get_parameter('pixel_pitch_y').value
        # FOV angles stored in radians for projection
        self.fov_horizontal_rad = math.radians(self.get_parameter('fov_horizontal').value)
        self.fov_vertical_rad = math.radians(self.get_parameter('fov_vertical').value)
        self.parent_frame = self.get_parameter('parent_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.tf_child_frame = self.get_parameter('tf_child_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.tf_x = self.get_parameter('tf_x').value
        self.tf_y = self.get_parameter('tf_y').value
        self.tf_z = self.get_parameter('tf_z').value
        self.tf_roll = self.get_parameter('tf_roll').value
        self.tf_pitch = self.get_parameter('tf_pitch').value
        self.tf_yaw = self.get_parameter('tf_yaw').value
        self.use_imu_orientation = self.get_parameter('use_imu_orientation').value
        imu_topic = self.get_parameter('imu_topic').value
        mag_topic = self.get_parameter('mag_topic').value
        madgwick_beta = self.get_parameter('madgwick_beta').value

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # IMU fusion state
        self.madgwick = MadgwickFilter(beta=madgwick_beta)
        self.latest_imu = None
        self.latest_mag = None
        self.orientation_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]

        # Subscriber for Argus
        self.subscription = self.create_subscription(
            ArgusResults,
            argus_topic,
            self.argus_callback,
            10
        )

        # IMU and Magnetometer subscribers (if enabled)
        if self.use_imu_orientation:
            self.imu_sub = self.create_subscription(
                Imu,
                imu_topic,
                self.imu_callback,
                10
            )
            self.mag_sub = self.create_subscription(
                MagneticField,
                mag_topic,
                self.mag_callback,
                10
            )
            self.get_logger().info(
                f'IMU orientation enabled. Subscribing to IMU: {imu_topic}, Mag: {mag_topic}'
            )

        # Publisher
        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(
            f'Argus to PointCloud node started. '
            f'Subscribing to: {argus_topic}, Publishing to: {output_topic}'
        )
        if self.publish_tf:
            self.get_logger().info(
                f'Publishing TF: {self.parent_frame} -> {self.tf_child_frame}'
            )

    def imu_callback(self, msg: Imu):
        """Store latest IMU data and update Madgwick filter."""
        self.latest_imu = msg

        # Extract gyro and accel
        gyro = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        accel = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]

        # Get magnetometer data if available
        mag = None
        if self.latest_mag is not None:
            mag = [
                self.latest_mag.magnetic_field.x,
                self.latest_mag.magnetic_field.y,
                self.latest_mag.magnetic_field.z
            ]

        # Get timestamp in seconds
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Update Madgwick filter
        self.orientation_quaternion = self.madgwick.update(gyro, accel, mag, timestamp)

    def mag_callback(self, msg: MagneticField):
        """Store latest magnetometer data."""
        self.latest_mag = msg

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (radians) to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def publish_transform(self, stamp):
        """Publish the TF transform from parent_frame to tf_child_frame."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.tf_child_frame

        # Translation
        t.transform.translation.x = self.tf_x
        t.transform.translation.y = self.tf_y
        t.transform.translation.z = self.tf_z

        # Rotation: use IMU orientation if enabled, otherwise use static Euler angles
        if self.use_imu_orientation:
            # orientation_quaternion is [w, x, y, z]
            q = self.orientation_quaternion
            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]
        else:
            # Rotation (from Euler angles)
            qx, qy, qz, qw = self.euler_to_quaternion(
                self.tf_roll, self.tf_pitch, self.tf_yaw
            )
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def rotate_point_by_quaternion(self, point, q):
        """
        Rotate a point [x, y, z] by quaternion [w, x, y, z].
        Returns rotated point as [x, y, z].
        """
        px, py, pz = point
        qw, qx, qy, qz = q

        # Quaternion rotation: q * p * q^-1
        # Using the formula for rotating a vector by a quaternion
        # v' = v + 2*qw*(qv x v) + 2*(qv x (qv x v))
        # where qv = [qx, qy, qz]

        # Cross product qv x v
        cx1 = qy * pz - qz * py
        cy1 = qz * px - qx * pz
        cz1 = qx * py - qy * px

        # Cross product qv x (qv x v)
        cx2 = qy * cz1 - qz * cy1
        cy2 = qz * cx1 - qx * cz1
        cz2 = qx * cy1 - qy * cx1

        # Result
        rx = px + 2 * (qw * cx1 + cx2)
        ry = py + 2 * (qw * cy1 + cy2)
        rz = pz + 2 * (qw * cz1 + cz2)

        return rx, ry, rz

    def argus_callback(self, msg: ArgusResults):
        """Convert ArgusResults to PointCloud2."""
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = msg.header.stamp

        # Use sensor_frame if the incoming frame_id is empty, otherwise use it
        if msg.header.frame_id:
            cloud_msg.header.frame_id = msg.header.frame_id
        else:
            cloud_msg.header.frame_id = self.sensor_frame

        # Publish TF if enabled
        if self.publish_tf:
            self.publish_transform(msg.header.stamp)

        # Define point fields: x, y, z, intensity (amplitude), snr, status
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='snr', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='status', offset=20, datatype=PointField.UINT8, count=1),
        ]

        # Point size in bytes (5 floats + 1 uint8 + 3 padding for alignment)
        point_step = 24
        cloud_msg.point_step = point_step
        cloud_msg.height = GRID_HEIGHT
        cloud_msg.width = GRID_WIDTH
        cloud_msg.row_step = point_step * GRID_WIDTH
        cloud_msg.is_dense = False
        cloud_msg.is_bigendian = False

        # Build point cloud data
        # Pixel ordering: [X0Y0, X0Y1, X0Y2, X0Y3, X1Y0, X1Y1, ..., X7Y3]
        # This means pixels are ordered by X first, then Y
        data = bytearray()

        num_pixels = len(msg.pixel)
        if num_pixels == 0:
            self.get_logger().warn('No pixels in ArgusResults message')
            return

        for idx in range(TOTAL_PIXELS):
            if idx < num_pixels:
                pixel = msg.pixel[idx]

                # Calculate grid position from flattened index
                # Index = x * GRID_HEIGHT + y (based on [X0Y0, X0Y1, ..., X7Y3])
                x_idx = idx // GRID_HEIGHT
                y_idx = idx % GRID_HEIGHT

                # Z is the range (depth)
                z_pos = pixel.range if not math.isnan(pixel.range) else 0.0

                # Calculate X, Y position in sensor frame (centered)
                # When Z=0, use pixel_pitch; when Z>0, project using FOV angles
                if z_pos < 0.0:
                    z_pos = 0.0
                x_angle = (x_idx - (GRID_WIDTH - 1) / 2.0) * self.fov_horizontal_rad
                y_angle = ((GRID_HEIGHT - 1) / 2.0 - y_idx) * self.fov_vertical_rad
                x_pos = ((x_idx - (GRID_WIDTH - 1) / 2.0) * self.pixel_pitch_x) + z_pos * math.tan(x_angle)
                y_pos = (((GRID_HEIGHT - 1) / 2.0 - y_idx) * self.pixel_pitch_y) + z_pos * math.tan(y_angle)

                # Intensity from amplitude
                intensity = pixel.amplitude

                # SNR (signal-to-noise ratio)
                snr = pixel.snr

                # Status
                status = pixel.status
            else:
                # Fill with NaN for missing pixels
                x_pos = float('nan')
                y_pos = float('nan')
                z_pos = float('nan')
                intensity = 0.0
                snr = 0.0
                status = 0

            # Pack point data (little-endian): 5 floats + 1 uint8 + 3 padding
            point_data = struct.pack('<fffffB3x', x_pos, y_pos, z_pos, intensity, snr, status)
            data.extend(point_data)

        cloud_msg.data = bytes(data)

        # Publish
        self.publisher.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArgusToPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
