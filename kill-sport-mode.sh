ssh pi@192.168.123.161 <<'ENDSSH'
sudo kill $(ps aux | grep 'sudo ./keep_sport_alive.sh' | awk '{print $2}')
sudo kill $(ps aux | grep '/bin/bash ./keep_sport_alive.sh' | awk '{print $2}')
sudo kill $(ps aux | grep './bin/Legged_sport' | awk '{print $2}')
ENDSSH

