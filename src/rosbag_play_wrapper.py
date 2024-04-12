#!/usr/bin/env python
import rospy
import subprocess

NAMESPACE = 'rosbag_play'

class RosbagPlayWrapper:
    def __init__(self, params={}, namespace=''):
        self.ns = namespace
        self.cmd = ["rosbag", "play"]
        self.params = params

        # construct cmd line string
        for param in self.params:
            self.get_param_and_extend_cmdline(param)

    def get_param_and_extend_cmdline(self, cmdline_option):
        """Fetch a parameter and extend the command-line list with it if necessary."""
        default_value = self.params[cmdline_option]
        # Construct the full command line option
        full_cmdline_option = "--" + cmdline_option

        # Fetch the parameter using rospy, with the namespace and default value
        value = rospy.get_param(f'{self.ns}/{cmdline_option}', default_value)
        if value:
            if isinstance(value, bool):
                # For boolean flags, add the option only if True
                if value:
                    self.cmd.append(full_cmdline_option)

            if isinstance(value, list):
                # For list items, handle them by extending with multiple entries
                self.cmd.extend([full_cmdline_option] + [str(v) for v in value])

            else:
                # For other types, add the option and its value
                self.cmd.extend([full_cmdline_option, str(value)])

    def execute(self):
        rospy.loginfo("Executing command: " + ' '.join(self.cmd))
        proc = subprocess.Popen(self.cmd)
        proc.wait()
        
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Command execution completed.")

def main():
    rospy.init_node('rosbag_play_wrapper')
    params = rospy.get_param(NAMESPACE) # get dictionary of params in namespace
    #print(params)

    wrapper = RosbagPlayWrapper(params=params, namespace=NAMESPACE)

    # Execute the command
    wrapper.execute()
    rospy.spin()
if __name__ == '__main__':
    main()
