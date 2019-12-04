#!/bin/bash

# requirements:
#     git

line_sep() {
    COLUMNS=`tput cols`
    printf '=%.0s' `eval echo {1..$COLUMNS}`
    echo
}

yes_or_no() {
    if [ -n "$2" ]; then
        echo -n 'yes/no: '
    elif [ -n "$1" ]; then
        echo -ne "$1: "
    fi
    case `read; echo $REPLY` in
        yes) return 1;;
        no)  return 0;;
        *)   yes_or_no "$1" REC;;
    esac
}

color_pallet() {
    for code in {0..15}; do
        echo -ne "\033[38;5;${code}m $(printf %03d $code)"
    done
    echo
    for code in {16..255}; do
        echo -ne "\033[38;5;${code}m $(printf %03d $code)"
        if [ $((($code - 16) % 12)) -eq 11 ]; then
            echo
        fi
    done
}

bashrc=~/.bashrc-alpaca-san

echo "# alias settings" > $bashrc
echo "alias sudo='sudo -E '" >> $bashrc
echo "alias pbcopy='xsel --clipboard --input'" >> $bashrc
echo "alias open='xdg-open'" >> $bashrc
echo >> $bashrc
alias sudo='sudo -E '

line_sep
sudo apt update
sudo apt upgrade -y
line_sep
sudo apt install -y vim net-tools openssh-server curl xsel apt-transport-https ca-certificates

line_sep
yes_or_no 'set proxy?' || {
    while true;do
        echo -n 'type proxy address (http://<host>:<port>): '
        read proxy
        yes_or_no "'$proxy' is ok?" || {
			# for apt
            echo "Acquire::http::Proxy \"$proxy\";" | sudo tee /etc/apt/apt.conf
            echo "Acquire::https::Proxy \"$proxy\";" | sudo tee -a /etc/apt/apt.conf
            echo "Acquire::ftp::Proxy \"$proxy\";" | sudo tee -a /etc/apt/apt.conf
            # for snap
			sudo snap set system proxy.http="$proxy"
			sudo snap set system proxy.https="$proxy"
			# for .bashrc
            echo '# proxy settings' >> $bashrc
            echo "export http_proxy=$proxy" >> $bashrc
            echo 'export https_proxy=$http_proxy' >> $bashrc
            echo 'export ftp_proxy=$http_proxy' >> $bashrc
            echo >> $bashrc
            export http_proxy=$proxy
	    export https_proxy=$proxy
	    export ftp_proxy=$proxy
	    break
        }
    done
}

line_sep
yes_or_no 'install pyenv?' || {
    if [ -d ~/.pyenv ]; then
        echo 'pyenv is already installed.'
    else
        echo 'no'
        sudo apt install -y libreadline-dev libffi-dev
        git clone http://github.com/yyuu/pyenv.git ~/.pyenv
    fi
    echo '# pyenv settings' >> $bashrc
    echo 'export PYENV_ROOT="$HOME/.pyenv"' >> $bashrc
    echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> $bashrc
    echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> $bashrc
    echo >> $bashrc
}

line_sep
yes_or_no 'setup prompt?' || {
    color_pallet
    while true;do
        echo -n 'type color number: '
        read ps_color
        if [ `expr "$ps_color" + 1 >/dev/null 2>&1 ; echo $?` -lt 2 ]; then
            yes_or_no "\033[38;5;${ps_color}mis this color ok?\033[0m" || {
                echo '# prompt settings' >> $bashrc
                echo "PS1_COLOR=${ps_color}" >> $bashrc
                echo "PS1='\${debian_chroot:+(\$debian_chroot)}\[\033[38;5;\${PS1_COLOR}m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\\$ '" >> $bashrc
                echo >> $bashrc
                break
            }
        fi
    done
}

line_sep
yes_or_no 'install VSCode?' || {
    if [ -n "$(which code)" ]; then
        echo 'VSCode is already installed.'
    else
        sudo snap install --classic code
    fi
}

line_sep
yes_or_no 'install n package system?' || {
	sudo apt install nodejs npm
	if [ -n "$http_proxy" ]; then
		sudo npm -g config set proxy "$http_proxy"
		sudo npm -g config set https-proxy "$https_proxy"
		sudo npm -g config set registry "http://registry.npmjs.org/"
	fi
	sudo npm install -g n
	sudo apt purge nodejs npm
}

line_sep
yes_or_no 'install docker?' || {
	# install docker
	sudo apt remove docker docker-engine docker.io containerd runc
	curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
	sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
	sudo apt update
	sudo apt install -y docker-ce docker-ce-cli containerd.io

	if [ -n $http_proxy ]; then
		# proxy settings for docker
		sudo mkdir -p /etc/systemd/system/docker.service.d
		echo -e "[Service]\nEnvironment=\"HTTP_PROXY=$http_proxy\""  | sudo tee /etc/systemd/system/docker.service.d/http-proxy.conf
		echo -e "[Service]\nEnvironment=\"HTTPS_PROXY=$https_proxy\"" | sudo tee /etc/systemd/system/docker.service.d/https-proxy.conf
		sudo systemctl daemon-reload
		sudo service docker restart 
	fi
}
if [ -f /opt/ros/melodic/setup.bash ]; then
    echo '# ROS-melodic settings' >> $bashrc
    echo 'source /opt/ros/melodic/setup.bash' >> $bashrc
    echo 'export TURTLEBOT3_MODEL=burger' >> $bashrc
    echo 'alias srcros="source devel/setup.bash"' >> $bashrc
    echo 'alias cm="catkin_make"' >> $bashrc
    echo >> $bashrc
fi

echo >> ~/.bashrc
echo "source $bashrc" >> ~/.bashrc
source ~/.bashrc

SCRIPT_DIR=$(cd $(dirname $0); pwd)
cp $SCRIPT_DIR/.inputrc ~/
cp $SCRIPT_DIR/.vimrc ~/
