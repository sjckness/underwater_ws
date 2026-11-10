# Ambiente ROS 2 per l'HIL con la work (Isaac Sim). Copiato sul Thor come ~/hil_env.sh. Uso:
#     source ~/hil_env.sh
# da un terminale dedicato: non tocca l'ambiente F1Tenth (dominio 0, discovery server 127.0.0.1:11811), che resta
# com'e' negli altri terminali. Il dominio 73 deve coincidere con quello di hil_env.bat sulla work.
source /opt/ros/jazzy/setup.bash
# F1Tenth puo' averli impostati: qui devono sparire, altrimenti il DDS resta su loopback o parla col discovery server
unset ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/hil_fastdds.xml"
echo "[hil] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  profilo=$FASTRTPS_DEFAULT_PROFILES_FILE"
