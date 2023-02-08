import tobii_research as tr
import rospy
from std_msgs.msg import Float32MultiArray
import logging
from datetime import datetime

# Link to Tobii Python SDK documentation
# https://developer.tobiipro.com/python/python-sdk-reference-guide.html
# https://developer.tobiipro.com/tobii.research/python/reference/1.10.2.17-alpha-g85317f98/examples.html

class TobiiGlasses:
    def __init__(self):

        # address = "tet-tcp://192.168.1.187"
        # self.eyetrackers = tr.EyeTracker(address)

        # print(type(self.eyetrackers))

        self.eyetrackers = tr.find_all_eyetrackers()

        print(type(self.eyetrackers))
        if len(self.eyetrackers) == 0:
            print("No eyetrackers found.")
            exit()

        for eyetracker in self.eyetrackers:
            print("Address: " + eyetracker.address)
            self.eyetracker_address = eyetracker.address
            print("Model: " + eyetracker.model)
            self.eyetracker_model = eyetracker.model
            print("Name (It's OK if this is empty): " + eyetracker.device_name)
            print("Serial number: " + eyetracker.serial_number)

        self.my_eyetracker = self.eyetrackers[0]
        print("Eyetracker found: " + self.my_eyetracker.product_id)

        self.test_calibration = tr.ScreenBasedCalibration(self.my_eyetracker)

    def calibrate(self):
        """
        Calibrates the Tobii Glasses Pro connected to the computer.
        This function starts the calibration process and waits for it to complete.
        The calibration data, status, start and end points will be printed to the console.
        """
        system_time_stamp = tr.get_system_time_stamp()
        print("The system time stamp in microseconds is {0}.".format(system_time_stamp))

        def on_calibration_started(calibration_status):
            print("Calibration started.")

        def on_calibration_failed(calibration_status):
            print("Calibration failed.")

        def on_calibration_success(calibration_status):
            print("Calibration succeeded.")

        def on_calibration_data(calibration_status):
            print("Calibration data: " + calibration_status.calibration_data)

        def on_calibration_type(calibration_type):
            print("Calibration type: " + calibration_type)

        def on_calibration_point_start(calibration_coordinates):
            print("Calibration point start: " + calibration_coordinates)

        def on_calibration_point_end(calibration_coordinates):
            print("Calibration point end: " + calibration_coordinates)

        calibration = tr.ScreenBasedCalibration(self.my_eyetracker)
        calibration.subscribe_to(tr.CALIBRATION_STARTED, on_calibration_started)
        calibration.subscribe_to(tr.CALIBRATION_FAILED, on_calibration_failed)
        calibration.subscribe_to(tr.CALIBRATION_SUCCESS, on_calibration_success)
        calibration.subscribe_to(tr.CALIBRATION_DATA, on_calibration_data)
        calibration.subscribe_to(tr.CALIBRATION_TYPE, on_calibration_type)
        calibration.subscribe_to(tr.CALIBRATION_POINT_START, on_calibration_point_start)
        calibration.subscribe_to(tr.CALIBRATION_POINT_END, on_calibration_point_end)

        # Show the calibration card to the user
        print("Please show the calibration card to the eyetracker.")

        # Start the calibration process
        calibration.start()

        # Wait for the calibration process to complete
        # Wait for the calibration process to complete
        calibration.wait_for_completion()

        # Unsubscribe from the calibration events
        calibration.unsubscribe_from(tr.CALIBRATION_STARTED, on_calibration_started)
        calibration.unsubscribe_from(tr.CALIBRATION_FAILED, on_calibration_failed)
        calibration.unsubscribe_from(tr.CALIBRATION_SUCCESS, on_calibration_success)
        calibration.unsubscribe_from(tr.CALIBRATION_DATA, on_calibration_data)
        calibration.unsubscribe_from(tr.CALIBRATION_TYPE, on_calibration_type)
        calibration.unsubscribe_from(tr.CALIBRATION_POINT_START, on_calibration_point_start)
        calibration.unsubscribe_from(tr.CALIBRATION_POINT_END, on_calibration_point_end)

        print(f"Finished calibrating model: {self.eyetracker_model} at adress: {self.eyetracker_address}")

    def start_streaming(self):
        """
        Start streaming gaze data and publishing it on a ROS topic 'tobii_gaze_data'.
        This function will start streaming gaze data and publishing it on a ROS topic 'tobii_gaze_data' using a publisher.
        It will be using the left gaze point data on display area.
        This function will work only after the calibration has been completed successfully
        """
        def gaze_data_callback(gaze_data):
            data = gaze_data["left_gaze_point_on_display_area"]
            gaze_data_publisher.publish(data)
            glasses.log_gaze_data(gaze_data)

        rospy.init_node('tobii_gaze_data_publisher')
        gaze_data_publisher = rospy.Publisher('tobii_gaze_data', Float32MultiArray, queue_size=10)
        self.my_eyetracker.subscribe_to(tr.EYETRACKER_GAZE_DATA, gaze_data_callback, as_dictionary=True)
        rospy.spin()

    def log_gaze_data(self, gaze_data):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        logging.debug('[{}] Received gaze data: {}'.format(timestamp, gaze_data))

    def test(self):
        """
        Test the calibration
        :return: the result of the calibration test
        """
        calibration_result = self.test_calibration.test()
        return calibration_result

def test_TobiiGlasses():
    glasses = TobiiGlasses()
    if len(glasses.eyetrackers) == 0:
        print("No eyetrackers found. Test failed.")
        return 0
    print("Eyetracker found: " + glasses.my_eyetracker.product_id)
    print("Starting calibration...")
    glasses.calibrate()
    print("Calibration completed. Testing if the calibration was successful...")
    # Check if the calibration was successful
    calibration_result = glasses.calibration.test()
    if calibration_result.result == tr.CALIBRATION_RESULT_FAILURE:
        print("Calibration failed. Test failed.")
        return 0
    print("Calibration was successful. Test passed.")
    return 1
# Dummy Main function to test the class and it's usage, uncomment for development
# Leave uncommented if Class should function as export module.

def main():
    test_success = test_TobiiGlasses()
    if test_success:
        print("Test was successful, glasses are connected as they should")
        glasses = TobiiGlasses()
        print("Proceeding with calibration...")
        glasses.calibrate()
        print("Strating the streaming of eyetracking data...")
        glasses.start_streaming()
    else:
        print("Something went wrong, check your glasses and connection again")

if __name__ == '__main__':
    main()
