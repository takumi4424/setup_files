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

line_sep
sudo apt update && sudo apt upgrade -y
line_sep
sudo apt install vim net-tools openssh-server curl xsel

line_sep
yes_or_no 'set proxy?' || {
    while true;do
        echo -n 'type proxy address (http://<host>:<port>): '
        read proxy
        yes_or_no "'$proxy' is ok?" || {
            echo "Acquire::http::Proxy \"$proxy\";" | sudo tee /etc/apt/apt.conf
            echo "Acquire::https::Proxy \"$proxy\";" | sudo tee -a /etc/apt/apt.conf
            echo "Acquire::ftp::Proxy \"$proxy\";" | sudo tee -a /etc/apt/apt.conf
            echo '# proxy settings' >> $bashrc
            echo "export http_proxy=$proxy" >> $bashrc
            echo 'export https_proxy=$http_proxy' >> $bashrc
            echo 'export ftp_proxy=$http_proxy' >> $bashrc
            echo >> $bashrc
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
        sudo apt install libreadline-dev libffi-dev
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
        curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > ./microsoft.gpg
        sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
        sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
        sudo apt update
        sudo apt install -y code
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
