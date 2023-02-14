import subprocess  # For executing a shell command

## Use system ping (probably linux only), to get the output and strip it to only the time in ms
# Returns ping to host in ms
def ping(host):

    #Example command in bash: 
    #       ping -c 1 host | awk -F '/' 'END {print $5}'
    
    command = ['ping', '-c', '1', host]
    regex_cmd = ['awk', "-F", '/', 'END {print $5}']

    p1 = subprocess.Popen(command, stdout=subprocess.PIPE)
    p2 = subprocess.Popen(regex_cmd, stdin=p1.stdout, stdout=subprocess.PIPE)

    out = p2.communicate()[0]
    readable_output = out.decode("utf-8").replace("\n", "")

    return readable_output

if __name__ == "__main__":

    for i in range(10):
        ping("192.168.100.1")