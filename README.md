# Multi_Agent_Project_1
After pulling a project down for the first time
Once you have pulled a committed project down from some remote git repository there are couple of things to be aware of.

1. If the project is a C++ project, find the .uproject file in explorer, right click and 'Generate Visual Studio project files'. Until you do this (or create a new code class from the editor) you will not have any visual studio solution to access.

2. Open the Epic Launcher and launch unreal. When you get the project selector click Browse and find your .uproject file.

3. Unreal will then prompt you to rebuild binaries. This will take a few minutes.

Once the binaries are built (and your solution files are generated if it is a C++ project) you should be good to start developing.

Note: While the editor plugin will continue to handle your content for you, if you are changing source files, creating new C++ classes, etc, you will need to have an external tool installed to check those files in. See Git source control (Tutorial) for links to external tools.