Java Test Runner
* Fix for empty test explorer
** https://github.com/Microsoft/vscode-java-test/issues/470#issuecomment-444681714
** Update .classpath file to add <attribute name="test" value="true" /> to attributes under "classpathentry kind=src" element
** NOTE:  Robot simulator will NOT run with this attribute enabled; comment out to enable simulator

Assumptions:

Positive values for driving move the robot forwards. Positive values for elevators move up. Positive values for things that 
rotate move clockwise. Always.
