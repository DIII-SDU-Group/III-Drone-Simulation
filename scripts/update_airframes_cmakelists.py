#!/usr/bin/python3

import os
import argparse

def main():
    parser = argparse.ArgumentParser(description='Update airframes CMakeLists.txt')
    
    # Arguments: path to CMakeLists.txt, list of airframes
    parser.add_argument('cmakelists', type=str, help='Path to CMakeLists.txt')
    parser.add_argument('airframes_folder', type=str, help='Folder containing airframes files')
    
    args = parser.parse_args()
    
    # Read CMakeLists.txt
    with open(args.cmakelists, 'r') as f:
        lines = f.readlines()
        
    # List contents of airframes folder
    airframes = os.listdir(args.airframes_folder)
        
    # The airframes are listed in the CMakeLists.txt file as follows:
    # px4_add_romfs_files(
    #     FILES
    #         ...
    #)
    
    # Add the airframes to the list of files before the closing parenthesis
    for airframe in airframes:
        # If the airframe is already listed, skip it
        already_listed = False
        for line in lines:
            if airframe == line.strip():
                already_listed = True
                break
        if already_listed:
            continue
        lines.insert(-1, "\t"+airframe+'\n')
        
    # Write the updated CMakeLists.txt file
    with open(args.cmakelists, 'w') as f:
        f.writelines(lines)
    
if __name__ == '__main__':
    main()
    