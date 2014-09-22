rosrun scitos_docking visual_charging_client charge 100
for i in $(seq 1 $1);
do 
a=$(rosrun scitos_docking visual_charging_client undock 100|tee -a testlog.txt |grep -c success);

if [ $a == 1 ]  
then
echo Undocking run $i OK;
else
echo Undocking run $i failed;
break
fi

a=$(rosrun scitos_docking visual_charging_client test 100|tee -a testlog.txt|grep -c success);
if [ $a == 1 ]  
then
echo Random run $i OK;
else
echo Random run $i failed;
break
fi

a=$(rosrun scitos_docking visual_charging_client charge 100|tee -a testlog.txt|grep -c success);
if [ $a == 1 ]  
then
echo Charging run $i OK;
else
echo Charging run $i failed;
break
fi

done
