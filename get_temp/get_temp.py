import paramiko
import time

hostname = '10.18.57.181'
port = 22
username = 'lzw365'
password = 'qwertyuiop'
remote_file_path = '/home/lzw365/Downloads/temp.txt'

ssh_client = paramiko.SSHClient()

ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

try:
    ssh_client.connect(hostname, port, username, password)
    time_stamp = ""
    while True:

        with ssh_client.open_sftp() as sftp:
            with sftp.file(remote_file_path, 'r') as remote_file:
                file_contents = remote_file.read().decode('utf-8')
                if time_stamp == "":
                    time_stamp = file_contents[:8]
                elif time_stamp != file_contents[:8]:
                    time_stamp = file_contents[:8]
                    print(file_contents[11:])
        time.sleep(2)

except Exception or KeyboardInterrupt as e:
    ssh_client.close()
    print("file read error!")