# print color pallet
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

