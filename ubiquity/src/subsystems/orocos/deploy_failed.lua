require "os"
os.execute("beep -f 280 -l 200 -r 8")
os.execute("sh /opt/ard/arp_core/script/linux/deployment_failed.sh")