# download images

function download_data() {
    url="https://labmgf.dica.polimi.it/pujob/cracks/images.zip"
    fname="img.tar.gz"
    wget --no-check-certificate --show-progress "$url$fname"
    rm "$fname"
    cd ../
}

download_data;

