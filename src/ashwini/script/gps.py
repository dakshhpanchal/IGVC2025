#!/usr/bin/env python3

import socket
import time
import gps
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 3764
SOCKET_TIMEOUT = 10  # seconds

def main():
    rospy.init_node("gps_corrector_node", anonymous=True)
    pub = rospy.Publisher("/gps", NavSatFix, queue_size=10)

    conn = None
    err_lat = 0.0
    err_lon = 0.0
    socket_active = False

    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.settimeout(SOCKET_TIMEOUT)
        server_socket.bind((LISTEN_IP, LISTEN_PORT))
        server_socket.listen(1)
        rospy.loginfo(f"[Server] Listening on {LISTEN_IP}:{LISTEN_PORT}")

        try:
            conn, addr = server_socket.accept()
            socket_active = True
            rospy.loginfo(f"[Server] Connection established with {addr}")
            conn.settimeout(1.0)
        except socket.timeout:
            rospy.logwarn(f"[Server] No client connected in {SOCKET_TIMEOUT}s, proceeding with zero error.")
            socket_active = False
    except Exception as e:
        rospy.logerr(f"[Server] Failed to start server: {e}")
        socket_active = False

    try:
        session = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
        rospy.loginfo("[GPS] Connected to gpsd. Waiting for GPS data...")

        while not rospy.is_shutdown():
            if socket_active:
                try:
                    data = conn.recv(1024).decode().strip()
                    if data:
                        err_lat_str, err_lon_str = data.split(",")
                        err_lat = float(err_lat_str)
                        err_lon = float(err_lon_str)
                except socket.timeout:
                    pass  # No new data, keep previous error values
                except Exception as e:
                    rospy.logwarn(f"[Recv] Error parsing data: {e}")
                    continue

            try:
                report = session.next()
                if report['class'] == 'TPV':
                    lat = getattr(report, 'lat', None)
                    lon = getattr(report, 'lon', None)
                    alt = getattr(report, 'alt', 0.0)

                    if lat is not None and lon is not None:
                        corrected_lat = lat - err_lat
                        corrected_lon = lon - err_lon

                        msg = NavSatFix()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = "gps_link"

                        msg.status.status = NavSatStatus.STATUS_FIX
                        msg.status.service = NavSatStatus.SERVICE_GPS

                        msg.latitude = corrected_lat
                        msg.longitude = corrected_lon
                        msg.altitude = alt

                        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                        pub.publish(msg)

                        rospy.loginfo(f"[Corrected GPS] Lat: {corrected_lat:.8f}, Lon: {corrected_lon:.8f}")
                        time.sleep(0.5)
                    else:
                        rospy.logwarn("[GPS] Waiting for valid GPS fix...")
            except KeyError:
                continue
            except StopIteration:
                rospy.logerr("[GPS] gpsd has terminated")
                break
            except Exception as e:
                rospy.logerr(f"[GPS] Error: {e}")
    except rospy.ROSInterruptException:
        rospy.loginfo("[Node] Shutting down.")
    except Exception as e:
        rospy.logerr(f"[Error] {e}")
    finally:
        if conn:
            conn.close()
        if 'server_socket' in locals():
            server_socket.close()
        rospy.loginfo("[Server] Connection closed.")

if __name__ == "__main__":
    main()
