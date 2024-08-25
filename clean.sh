rosnode kill --all
killall gzclient
killall gzserver
rosnode list | grep -v rosout | xargs rosnode kill