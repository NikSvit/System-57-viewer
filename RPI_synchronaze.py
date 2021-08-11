# Проверка синхронизации Raspberry Pi с NTP сервером
import subprocess

reply = subprocess.run(['timedatectl', 'status'], stdout = subprocess.PIPE, encoding='utf-8')

if reply.returncode == 0:
    print(reply.stdout)
    if reply.stdout.find("System clock synchronized: yes") != -1:
        print('Bingo!')
else:
    print('Unreachable')