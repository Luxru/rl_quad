# install
1. git clone 
2. create a catkin workspace and the `catkin init`
3. `catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
4. `catkin build` && `jq -s 'map(.[])' build/**/compile_commands.json > build/compile_commands.json`(for clangd detect compile files)