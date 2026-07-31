from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
