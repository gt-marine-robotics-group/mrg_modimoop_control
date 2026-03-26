class SailbotLogger(Node):
    def __init__(self):
        super().__init__('sailbot_logger')

        self.latest_imu = None
        self.latest_gps = None
        self.latest_mag = None
        self.latest_heading = None
        self.latest_cmd_rudder = None
        self.latest_cmd_wingsail = None

        self.log_path = ...
        self.buffer = []

        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(NavSatFix, '/gps', self.gps_cb, 10)
        self.create_subscription(MagneticField, '/magnetic_field', self.mag_cb, 10)
        self.create_subscription(Float64, '/heading_est', self.heading_cb, 10)

        self.timer = self.create_timer(0.1, self.log_timer_cb)