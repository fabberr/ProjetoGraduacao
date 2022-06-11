#!/usr/bin/env bash

#
# Author: Fabrício Milanez https://github.com/FabricioMilanez
# Setup script for scene reconstruction pipeline of https://github.com/FabricioMilanez/ProjetoGraduacao.git
#

exec_pipeline_sh_help() {
	echo "Usage: exec_pipeline.sh [OPTIONS] <dataset_name>"
	echo
	echo "    Sets the necessary environment variables and creates the output directory"
	echo "    for executing the full scene reconstruction pipeline."
	echo
	echo "    This script sets the following environment variables for its shell session:"
	echo "      - \$f2d             Feature detector and descriptor."
	echo "      - \$dataset_base    Base dataset path."
	echo "      - \$dataset_name    Name of dataset's root directory."
	echo "      - \$dataset_path    Path do image dataset. Derided from \"\$dataset_base/\$dataset_name\"."
	echo "      - \$output_base     Base output path."
	echo "      - \$output_dir      Path to output directory. Derided from \"\$output_base/\$dataset_name/\$f2d\"."
	echo "      - \$MVS_root        Installation path of OpenMVS's apps."
	echo
	echo "OPTIONS"
	echo "    -h                   Display this help message and exit."
	echo "    -f <f2d>             Feature detector and descriptor to use during sparse"
	echo "                         point cloud generation. Valid options are:"
	echo "                           - \"AKAZE\" (default)"
	echo "                           - \"SIFT\""
	echo "                           - \"SURF\""
	echo "                           - \"ORB\""
	echo "    -d <dataset_base>    Base dataset path, \"./datasets\" by default."
	echo "    -o <output_base>     Base output path, \"./output/pipeline\" by default."
	echo "    -r <OpenMVS_root>    Installation path of DensifyPointCloud, ReconstructMesh,"
	echo "                         RefineMesh and TextureMesh. \"/usr/local/bin/OpenMVS\", by default."
	echo "    -y                   Do not prompt to continue after environment review."
	echo
}

# Parsing optional args
while getopts hf:d:o:r:y opt
do
    case "${opt}" in
		h) # display help
			exec_pipeline_sh_help
			exit 0;;
        f) export f2d=${OPTARG};;
		d) export dataset_base=${OPTARG};;
		o) export output_base=${OPTARG};;
		r) export OpenMVS_root=${OPTARG};;
		y) prompt=true;;
    esac
done
shift $((OPTIND-1))

# Ensure optional args have a default value if not present
if ! [ "$f2d" ]; then
	export f2d="AKAZE"
fi

if ! [ "$dataset_base" ]; then
	export dataset_base="./datasets"
fi

if ! [ "$output_base" ]; then
	export output_base="./output/pipeline"
fi

if ! [ "$OpenMVS_root" ]; then
	export OpenMVS_root="/usr/local/bin/OpenMVS"
fi

# Parsing non-optional args
if ! [ "$1" ]; then
	echo "Non-optional argument <dataset_name> not present, terminating."
	echo
	exec_pipeline_sh_help
	exit 1
fi
export dataset_name=$1

# Determining input and output path variables
export dataset_path="$dataset_base/$dataset_name"
export output_dir="$output_base/$dataset_name/$f2d"

# Review
echo "vars={"
echo "    f2d=$f2d"
echo "    dataset_path=$dataset_path"
echo "    output_dir=$output_dir"
echo "    OpenMVS_root=$OpenMVS_root"
echo "}"

if [ -v $prompt ]; then
	read -r -p "Proceed? [Y/n] " input
	case $input in
		[Nn][oO] | [Nn])
			echo "Cancelled by user."
			exit 0;;
	esac
fi

# Create output directories
mkdir -p $output_dir;

# Create symlink to built executables if they don't already exist
bins=( packt-sfm mesh )
for bin in "${bins[@]}"
do
	if ! [ -x "./$bin" ]; then
		ln -s "build/$bin/$bin" "$bin"
	fi
done

#
# Pipeline Execution
#

# Sparse cloud generation and camera poses estimation
printf '\n[PIPELINE] Computing sparse point cloud and estimating camera poses\n\n'
{ time ./packt-sfm --f2d=$f2d --cloud="$output_dir/sparse.obj" --mvs="$output_dir/sparse.mvs" $dataset_path > "$output_dir/sparse_output.txt" 2>&1; } 2> "$output_dir/sparse_time.txt"

# Dense cloud generation
printf '\n[PIPELINE] Computing dense point cloud\n\n'
$OpenMVS_root/DensifyPointCloud -i "$output_dir/sparse.mvs" -o "$output_dir/dense"

# OpenMVS mesh reconstruction and texturing
printf '\n[PIPELINE] Reconstructing mesh (OpenMVS)\n\n'
$OpenMVS_root/ReconstructMesh -i "$output_dir/dense.mvs" -o "$output_dir/mesh"
$OpenMVS_root/TextureMesh --export-type ply -i "$output_dir/mesh.mvs" -o "$output_dir/mesh_textured"
rm ./*.dmap

# PCL Greedy Projection Triangulation and Poisson mesh reconstruction
printf '\n[PIPELINE] Reconstructing mesh (pcl::GreedyProjectionTriangulation)\n\n'
./mesh "$output_dir/dense.ply"
printf '\n[PIPELINE] Reconstructing mesh (pcl::Poisson)\n\n'
./mesh "$output_dir/dense.ply" --method=POISSON
