# alias settings
alias sudo='sudo -E '
alias less="vim --cmd 'let no_plugin_maps = 1' -c 'runtime! macros/less.vim'"

if [ "$(uname -s)" == 'Linux' ]; then
  if grep '^NAME="Ubuntu' /etc/os-release >/dev/null ; then
    alias pbcopy='xsel --clipboard --input'
    alias open='xdg-open'
  fi
fi

