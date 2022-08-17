# the name of the bag is the timestemp when the bag is recorded
BAG_NAME="$(date +'%d-%m-%Y_%Ih%Mm%S')"

# start recording the bag
rosbag record -a -O ${BAG_NAME}