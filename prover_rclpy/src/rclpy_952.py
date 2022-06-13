import rclpy.time
import rclpy.duration

t = rclpy.time.Time()
print("t in [None, t]: %r" % (t in [None, t]))

d = rclpy.duration.Duration()
print("d in [None, d]: %r" % (d in [None, d]))