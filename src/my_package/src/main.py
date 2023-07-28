#!/usr/bin/env python
from dwm1001_systemDefinitions import SYS_DEFS

__author__     = SYS_DEFS.AUTHOR
__version__    = SYS_DEFS.VERSION
__maintainer__ = SYS_DEFS.MAINTAINER
__email__      = SYS_DEFS.EMAIL
__status__     = SYS_DEFS.STATUS


import rospy, time, serial, os
from geometry_msgs.msg import PointStamped
from dwm1001_apiCommands            import DWM1001_API_COMMANDS
from dynamic_reconfigure.server     import Server
from my_package.cfg          import DWM1001_Tune_SerialConfig
from my_package.msg          import Anchor
from my_package.msg          import Tag
from my_package.srv         import Anchor_0



rospy.init_node('my_package', anonymous=False)
os.popen("sudo chmod 777 /dev/ttyACM0", "w")
rate = rospy.Rate(1)
serialReadLine = ""
dynamicConfig_OPEN_PORT = {"open_port": False}
dynamicConfig_CLOSE_PORT = {"close_port": False}
dynamicConfig_SERIAL_PORT = {"serial_port": ""}
serialPortDWM1001 = serial.Serial(
    port       = str(rospy.get_param('~serial_port_name')),
    baudrate   = int(rospy.get_param('~serial_baud_rate')),
    parity=SYS_DEFS.parity,
    stopbits=SYS_DEFS.stopbits,
    bytesize=SYS_DEFS.bytesize
)


class dwm1001_localizer:
    def main(self):
        global serialReadLine
        serialPortDWM1001.close()
        time.sleep(1)
        serialPortDWM1001.open()
        if(serialPortDWM1001.isOpen()):
            self.initializeDWM1001API()
            time.sleep(2)
            serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001 coordinates")
        else:
            rospy.loginfo("Can't open port: "+ str(serialPortDWM1001.name))
        try:
            while not rospy.is_shutdown():
                serialReadLine = serialPortDWM1001.read_until()
                try:
                    self.pubblishCoordinatesIntoTopics(self.splitByComma(serialReadLine))
                except IndexError:
                    rospy.loginfo("Found index error in the network array!DO SOMETHING!")
        except KeyboardInterrupt:
            rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")
            serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rate.sleep()
            if "reset" in serialReadLine:
                rospy.loginfo("succesfully closed ")
                serialPortDWM1001.close()

    def splitByComma(self, dataFromUSB):
        arrayFromUSBFormatted = [x.strip() for x in dataFromUSB.strip().split(',')]
        return arrayFromUSBFormatted

    def __init__(self):
        self.anchor_pub = rospy.Publisher('/dwm1001/anchor', Anchor, queue_size=10)
        self.tag_pub = rospy.Publisher('/dwm1001/tag', Tag, queue_size=10)
        self.geo_pub = rospy.Publisher('/point', PointStamped, queue_size=10)
    def pubblishCoordinatesIntoTopics(self, networkDataArray):
        for network in networkDataArray:
            if 'AN' in network:
                temp_anchor_number = networkDataArray[networkDataArray.index(network)]
                anchor = Anchor(  str(networkDataArray[networkDataArray.index(network) + 1]),
                                float(networkDataArray[networkDataArray.index(network) + 2]),
                                float(networkDataArray[networkDataArray.index(network) + 3]),
                                float(networkDataArray[networkDataArray.index(network) + 4]),
                                float(networkDataArray[networkDataArray.index(network) + 5]))
                # publish each anchor, add anchor number to the topic, so we can pubblish multiple anchors
                # example /dwm1001/anchor0, the last digit is taken from AN0 and so on
                rospy.loginfo("Anchor: "
                              + str(anchor.id)
                              + " x: "
                              + str(anchor.x)
                              + " y: "
                              + str(anchor.y)
                              + " z: "
                              + str(anchor.z))
                self.anchor_pub.publish(anchor)
            elif 'POS' in network:
                tag = Tag(float(networkDataArray[networkDataArray.index(network) + 1]),
                          float(networkDataArray[networkDataArray.index(network) + 2]),
                          float(networkDataArray[networkDataArray.index(network) + 3]),)
                # publish tag
                rospy.loginfo("Tag: "
                              + " x: "
                              + str(tag.x)
                              + " y: "
                              + str(tag.y)
                              + " z: "
                              + str(tag.z))
                self.tag_pub.publish(tag)
                point = PointStamped()
                point.header.stamp = rospy.Time.now()
                point.point.x = tag.x
                point.point.y = tag.y
                point.point.z = tag.z
                self.geo_pub.publish(point)
    def updateDynamicConfiguration_SERIALPORT(self):
        global dynamicConfig_SERIAL_PORT
        dynamicConfigServer = Server(DWM1001_Tune_SerialConfig, self.callbackDynamicConfig)
        dynamicConfig_CLOSE_PORT.update({"close_port": True})
        dynamicConfig_OPEN_PORT.update({"open_port" : False})
        dynamicConfigServer.update_configuration(dynamicConfig_OPEN_PORT)
        dynamicConfigServer.update_configuration(dynamicConfig_CLOSE_PORT)
        dynamicConfig_CLOSE_PORT.update({"close_port": False})
        dynamicConfig_OPEN_PORT.update({"open_port": True})
        dynamicConfig_SERIAL_PORT = {"serial_port": str(serialPortDWM1001.name)}
        dynamicConfigServer.update_configuration(dynamicConfig_OPEN_PORT)
        dynamicConfigServer.update_configuration(dynamicConfig_OPEN_PORT)
        dynamicConfigServer.update_configuration(dynamicConfig_CLOSE_PORT)
        dynamicConfigServer.update_configuration(dynamicConfig_SERIAL_PORT)

    def initializeDWM1001API(self):
        serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

    def callbackDynamicConfig(self, config, level):
        global serialReadLine
        if config["quit_dwm1001_api"]:
            rospy.loginfo("Not implement it yet")
            config["quit_dwm1001_api"] = False

        if config["close_port"]:
            rospy.loginfo("Close port not implement it yet")
            config["close_port"] = False

        if config["exit"]:
            rospy.loginfo("Not implement it yet")
            config["exit"] = False

        return config

def start():
    dwm1001 = dwm1001_localizer()
    dwm1001.main()


if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
