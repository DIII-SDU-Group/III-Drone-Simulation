#!/usr/bin/python3

import os
import argparse
import copy

def main():
    parser = argparse.ArgumentParser(description='Update SITL targets CMakeLists.txt')
    
    parser.add_argument('cmakelists', type=str, help='Path to CMakeLists.txt')
    parser.add_argument('models_folder', type=str, help='Folder containing model files')
    parser.add_argument('world_models_folder', type=str, help='Folder containing world model files')
    parser.add_argument("worlds_folder", type=str, help='Folder containing world files')
    
    args = parser.parse_args()
    
    # Read CMakeLists.txt
    with open(args.cmakelists, 'r') as f:
        lines = f.readlines()
        
    models = os.listdir(args.models_folder)
    models += os.listdir(args.world_models_folder)
    worlds = os.listdir(args.worlds_folder)
    for i in range(len(worlds)):
        # Remove the .world extension
        worlds[i] = worlds[i].split('.')[0]
        
    
    # Find the indices of the set(models line and the following closing parenthesis
    set_models_index = -1
    closing_parenthesis_index = -1
    
    for i in range(len(lines)):
        if 'set(models' in lines[i]:
            set_models_index = i
        if ')' in lines[i] and set_models_index != -1:
            closing_parenthesis_index = i
            break
        
    models_lines = copy.deepcopy(lines[set_models_index+1:closing_parenthesis_index])
    
    model_lines_to_add = []
    
    for model in models:
        # If the model is already listed, skip it
        already_listed = False
        for line in models_lines:
            if model == line.strip():
                already_listed = True
                break
        if already_listed:
            continue
        model_lines_to_add.append('\t\t'+model+'\n')
    
    lines = lines[:closing_parenthesis_index] + model_lines_to_add + lines[closing_parenthesis_index:]
    
    # Find the indices of the set(worlds line and the following closing parenthesis
    set_worlds_index = -1
    closing_parenthesis_index = -1
    
    for i in range(len(lines)):
        if 'set(worlds' in lines[i]:
            set_worlds_index = i
        if ')' in lines[i] and set_worlds_index != -1:
            closing_parenthesis_index = i
            break
        
    worlds_lines = copy.deepcopy(lines[set_worlds_index+1:closing_parenthesis_index])
    
    world_lines_to_add = []
    
    for world in worlds:
        # If the world is already listed, skip it
        already_listed = False
        for line in worlds_lines:
            if world == line.strip():
                already_listed = True
                break
        if already_listed:
            continue
        world_lines_to_add.append('\t\t'+world+'\n')

    lines = lines[:closing_parenthesis_index] + world_lines_to_add + lines[closing_parenthesis_index:]
        
    # Write the updated CMakeLists.txt file
    with open(args.cmakelists, 'w') as f:
        f.writelines(lines)
    
if __name__ == '__main__':
    main()
    