#CP2102 ���ں�0001 ���ñ���Ϊwheeltec_lidar
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar.rules
#CH9102��ͬʱϵͳ��װ�˶�Ӧ���� ���ں�0001 ���ñ���Ϊwheeltec_lidar
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="54B8001974", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar2.rules
#CH9102��ͬʱϵͳû�а�װ��Ӧ���� ���ں�0001 ���ñ���Ϊwheeltec_lidar
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar3.rules

service udev reload
sleep 2
service udev restart


