#!/usr/bin/env bash

find . -type f -name '*.gltf' -print0 | xargs -0 -I {} -P 8 sh -c 'gltfpack -i "$1" -o "${1%.gltf}.glb" -noq -kn -km' _ {}
mv assets/MedievalVillage/glTF/*.glb assets/MedievalVillage/glb
