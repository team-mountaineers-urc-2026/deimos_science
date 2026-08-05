import seabreeze
seabreeze.use('pyseabreeze')
from seabreeze.spectrometers import Spectrometer
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import StringArray, SpectrometerData
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from std_msgs.msg import Empty
import time, os, csv
import numpy as np

# Install the following two commands: 
# pip install seabreeze[pyseabreeze]
# seabreeze_os_setup

# Addtionally, add user to dialout and tty groups using:
# sudo usermod -a -G tty username_here
# sudo usermod -a -G dialout username_here

class SpectrometerCalibrate(Node):

    def __init__(self):
        super().__init__('spectrometer_calibrate_node')
        self.declare_parameter('serial_number', "S14413")

        # self.spectrometer_service = self.create_service(
        #     srv_type= CollectSpectrometerData,
        #     srv_name= self.get_parameter('service_name').get_parameter_value().string_value,
        #     callback= self.collect_data
        # )


        # Parameters
        self.declare_parameter('data_folder', os.path.expanduser("~") + '/workspace-deimos/src/science/deimos_science/spectrometer_data_calibrate')
        self.declare_parameter('integration_time', 10_000)

        self.integrate_time = self.get_parameter('integration_time').get_parameter_value().integer_value
        self.data_folder = self.get_parameter('data_folder').get_parameter_value().string_value


        self.filename = 'spect_calibration.csv'

        # Subscribers
        self.request_info_sub = self.create_subscription(Int32, 'spectrometer/request', self.collect_data, 10)
        self.calibrate_spec_sub = self.create_subscription(Int32, 'spectrometer/calibrate', self.calibrate_spectrometer, 10)
        # Receive confirmation that halogen bulb is on or off
        # calibrate data (Action service??)

        # Publishers
        self.send_info_pub = self.create_publisher(SpectrometerData, 'spectrometer/result', 10)
        # Publish commands to halogen bulb


    def collect_data(self, msg):
        self.integrate_time = msg.data

        sn = self.get_parameter('serial_number').get_parameter_value().string_value

        response = SpectrometerData()

        try:
            spec = Spectrometer.from_serial_number(sn)
        except Spectrometer._backend.SeaBreezeError:
            try:
                self.get_logger().error(f'Failed to open spectrometer by the provided serial number {sn}. Trying the first available.')
                spec = Spectrometer.from_first_available()
            except Spectrometer._backend.SeaBreezeError:
                self.get_logger().error(f'No spectrometers were available')
                raise Exception("Could not find a spectrometer connected.")

        try:
            spec.integration_time_micros(self.integrate_time)
            response.wavelengths = list(spec.wavelengths())
            response.intensities = list(spec.intensities())
            response.is_successful = True
        except Exception:
            self.get_logger().info(f'Integration Time "{self.integrate_time}" was invalid. Must be between 10-85,000,000')
            response.wavelengths = []
            response.intensities = []
            response.is_successful = False

        # Read Callibration CSV 
        absorbance = []
        calibration_data =  []
        
        with open(f'{self.data_folder}/{self.filename}', mode='r') as file:
            reader = csv.reader(file)
            for row in reader:
                calibration_data.append([float(x) for x in row])

        wavelengths = calibration_data[0]
        dark = calibration_data[1]
        light = calibration_data[2]
        sample = response.intensities

        # Calculate Absorbance
        if np.allclose(wavelengths, response.wavelengths):
            self.get_logger().info("Wavelengths are equal")
        
        sample = np.array(sample, dtype=float)
        dark = np.array(dark, dtype=float)
        light = np.array(light, dtype=float)

        # Avoid divide-by-zero / negative values
        eps = 1e-12

        numerator = sample - dark
        denominator = light - dark

        # Clip to avoid invalid values
        denominator = np.clip(denominator, eps, None)
        ratio = np.clip(numerator / denominator, eps, None)
        absorbance = -np.log10(ratio)

        absorbance = np.nan_to_num(absorbance, nan=0.0, posinf=0.0, neginf=0.0)

        response.intensities = absorbance.tolist()
        response.wavelengths = wavelengths

        # Publish the data 5 times to make sure we received it
        self.send_info_pub.publish(response)



    def calibrate_spectrometer(self, msg):
        # Setup Spectrometer for Use
        sn = self.get_parameter('serial_number').get_parameter_value().string_value

        try:
            spec = Spectrometer.from_serial_number(sn)
        except Spectrometer._backend.SeaBreezeError:
            try:
                self.get_logger().error(f'Failed to open spectrometer by the provided serial number {sn}. Trying the first available.')
                spec = Spectrometer.from_first_available()
            except Spectrometer._backend.SeaBreezeError:
                self.get_logger().error(f'No spectrometers were available')
                raise Exception("Could not find a spectrometer connected.")
            
    
        # Grab previous calibration settings
        previous_data = []
        with open(f'{self.data_folder}/{self.filename}', mode='r') as file:
            reader = csv.reader(file)
            for row in reader:
                previous_data.append([float(x) for x in row])      


        # Use a fixed integration time for all acquisitions.
        results_dark = SpectrometerData()
        results_light = SpectrometerData()

        # Read dark room settings
        if(msg.data == 0):
            try:
                spec.integration_time_micros(self.integrate_time)
                results_dark.wavelengths = list(spec.wavelengths())
                results_dark.intensities = list(spec.intensities())
                results_dark.is_successful = True
                results_light.intensities = previous_data[2]
            except Exception:
                self.get_logger().info(f'Integration Time "{self.integrate_time}" was invalid. Must be between 10-85,000,000')
                results_dark.wavelengths = previous_data[0]
                results_dark.intensities = previous_data[1]
                results_dark.is_successful = False

        # Read Light Room Settings
        elif(msg.data == 1):
            try:
                spec.integration_time_micros(self.integrate_time)
                results_light.wavelengths = list(spec.wavelengths())
                results_light.intensities = list(spec.intensities())
                results_light.is_successful = True
                results_dark.wavelengths = previous_data[0]
                results_dark.intensities = previous_data[1]

            except Exception:

                self.get_logger().info(f'Integration Time "{self.integrate_time}" was invalid. Must be between 10-85,000,000')
                results_light.wavelengths = previous_data[0]
                results_light.intensities = previous_data[2]
                results_light.is_successful = False        


        # Turn on Halogen Bulb
        

        # Verify Turn-on

        # Read Light Value Settings

        #Save to CSV
        self.save_calibrate_data(results_dark, results_light)

        #float64[] wavelengths
        #float64[] intensities
        #bool is_successful


    ## CALIBRATION FILE CREATION ##
    def save_calibrate_data(self, dark: SpectrometerData, light : SpectrometerData):
        if not dark.is_successful and not light.is_successful:
            self.get_logger().error("Spectrometer was not successful :(")
            return

        # timestamp ??

        # Make the CSV
        data = [dark.wavelengths, dark.intensities, light.intensities]

        file = open(f'{self.data_folder}/{self.filename}', 'w+')
        writer = csv.writer(file)
        writer.writerows(data)
        file.close()

    #     self.send_folder()

    # ## CALIBRATION FILE STORAGE ## 
    # def send_folder(self, msg = Empty()):

    #     # If the folder doesn't exists, make it
    #     if not os.path.exists(self.data_folder):
    #         os.makedirs(self.data_folder)

    #     names = []
    #     for object in os.listdir(self.data_folder):
    #         if '.csv' in object: names.append(object)

    #     self.folder_publisher.publish(StringArray(data=names))

def main(args=None):

    rclpy.init(args=args)
    spectrometer_calibrate = SpectrometerCalibrate()
    rclpy.spin(spectrometer_calibrate)
    spectrometer_calibrate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()