CAN_PDO_1_ID=0x321

rtcansend rtcan1 -i 000 0x82 0x00 -v && sleep 1
rtcansend rtcan1 -i 000 0x01 0x00 -v && sleep 1
rtcansend rtcan1 -i 0x321 0xFD 0xFF 0xFF 0xFF 0xFF&& sleep 1
rtcansend rtcan1 -i 0x321 0x0F 0X0 0X0 0X0 0X0 -v && sleep 1
rtcansend rtcan1 -i 0x321 0x80 0Xce 0X09 0X0 0X0 -v && rtcansend rtcan1 -i 0x321 0x81 0XCE 0X09 0X0 0X0 -v
rtcansend rtcan1 -i 0x321 0x93 0XF4 0X01 0X0 0X0 -v && sleep 1
echo "Finish"
