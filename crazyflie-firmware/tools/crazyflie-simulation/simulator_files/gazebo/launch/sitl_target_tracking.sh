#!/bin/bash
function cleanup() {
	pkill -x cf2
	pkill -9 ruby
}

function spawn_model() {
	MODEL=$1
	N=$2 # Cf ID
	X=$3 # spawn x position
	Y=$4 # spawn y position
	X=${X:=$X}
	Y=${Y:=$Y}
	SUPPORTED_MODELS=("crazyflie", "crazyflie_thrust_upgrade")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL"* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi

	working_dir="$build_path/$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null


	set --
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/launch/jinja_gen.py
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/models/${MODEL}/model.sdf.jinja
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo
	set -- ${@} --cffirm_udp_port $((19950+${N}))
	set -- ${@} --cflib_udp_port $((19850+${N}))
	set -- ${@} --cf_id $((${N}))
	set -- ${@} --cf_name cf
	set -- ${@} --output-file /tmp/${MODEL}_${N}.sdf

	python3 ${@}

	echo "Spawning ${MODEL}_${N} at ${X} ${Y}"

    gz service -s /world/${world}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 --req 'sdf_filename: "/tmp/'${MODEL}_${N}'.sdf", pose: {position: {x:'${X}', y:'${Y}', z: 0.5}}, name: "'${MODEL}_${N}'", allow_renaming: 1'
	
	echo "starting instance $N in $(pwd)"
	$build_path/cf2 $((19950+${N})) > out.log 2> error.log &

	popd &>/dev/null
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Description: This script is used to spawn target and pursuer vehicles in gazebo."
	echo "Usage: $0 [-t <num_targets>] [-p <num_pursuers>] [-m <vehicle_model>] [-w <world>]"
	echo "Note: enable cf_1 ... cf_N (N = targets + pursuers) in ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml"
	exit 1
fi

# crazyflie ids are first target ids and then pursuer ids

# options are 
# -t: number of targets
# -p: number of pursuers
# -m: vehicle model
# -w: world type
while getopts t:p:m:w: option
do
	case "${option}"
	in
		t) NUM_TARGETS=${OPTARG};;
        p) NUM_PURSUERS=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
	esac
done

NUM_TARGETS=${NUM_TARGETS:=0}
NUM_PURSUERS=${NUM_PURSUERS:=1}
num_vehicles=$((NUM_TARGETS + NUM_PURSUERS))
world=${WORLD:=crazysim_default}
target=${TARGET:=cf2}
vehicle_model=${VEHICLE_MODEL:="crazyflie"}
export CF2_SIM_MODEL=gz_${vehicle_model}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../../../.."

build_path=${src_path}/sitl_make/build

echo "killing running crazyflie firmware instances"
pkill -x cf2 || true

sleep 1

source ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash ${src_path} ${build_path}

echo "Starting gazebo"
gz sim -s -r ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/worlds/${world}.sdf -v 0 &
sleep 3

if [ $num_vehicles -gt 8 ]
then
	echo "Tried spawning $num_vehicles vehicles. The maximum number of total vehicles is 8"
	exit 1
fi

# crazyflie ids are first target ids and then pursuer ids
# (cf_1 ... cf_T are targets, cf_T+1 ... cf_N are pursuers)
n=0
while [ $n -lt $num_vehicles ]; do
	# for target vehicles first
	if [ $n -lt $NUM_TARGETS ]
	then
		x_cord=$n
		y_cord=1.0
	else
		x_cord=$(($n - $NUM_TARGETS))
		y_cord=0.0
	fi

	spawn_model ${vehicle_model} $(($n)) $x_cord $y_cord
	n=$(($n + 1))
done

# The crazyflies.yaml config is NOT modified by this script
echo ""
echo "Spawned $num_vehicles crazyflie(s): $NUM_TARGETS target(s) and $NUM_PURSUERS pursuer(s)."
if [ $num_vehicles -eq 1 ]; then CF_LIST="cf_1"; else CF_LIST="cf_1 ... cf_$num_vehicles"; fi
echo "Make sure ONLY $CF_LIST have 'enabled: true' in"
echo "ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml before starting the crazyflie_server."
echo ""

trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo gui"
gz sim -g

# echo "Simulation running headless. Press Ctrl+C to exit."
# wait
