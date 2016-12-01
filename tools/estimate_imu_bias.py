import sys
import rospy
from sensor_msgs.msg import Imu

class EstimateImuBias:
    def __init__(self):
        self.data = []

    def callback(self, data):
        if len(self.data) == 0:
            print("Started ...")
        self.data.append(data)

    def print_results(self):
        print("RESULTS: ")
        accel_x, accel_y, accel_z = [], [], []
        gyro_x, gyro_y, gyro_z = [], [], []
        for data in self.data:
            accel_x.append(data.linear_acceleration.x)
            accel_y.append(data.linear_acceleration.y)
            accel_z.append(data.linear_acceleration.z)
            gyro_x.append(data.angular_velocity.x)
            gyro_y.append(data.angular_velocity.y)
            gyro_z.append(data.angular_velocity.z)

        gravity = (0.0, 0.0, 9.81)

        print("Imu.accelerometerBias: [{} {} {}]".format(sum(accel_x)/len(accel_x) - gravity[0], sum(accel_y)/len(accel_y) - gravity[1], sum(accel_z)/len(accel_z) - gravity[2]))
        print("Imu.gyroscopeBias: [{} {} {}]".format(sum(gyro_x)/len(gyro_x), sum(gyro_y)/len(gyro_y), sum(gyro_z)/len(gyro_z)))

    def run(self):
        rospy.init_node('estimate_imu_bias')
        print("Subscribing to {}".format(sys.argv[1]))
        rospy.Subscriber(sys.argv[1], Imu, self.callback)
        rospy.spin()
        self.print_results()

if __name__ == '__main__':
    est = EstimateImuBias()
    est.run()
