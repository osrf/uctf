#!/bin/bash

set -o errexit

SCRIPTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

PACKAGING_DIR=`mktemp -d `

LOCAL_DIR=`pwd`

# deletes the temp directory
function cleanup {
  rm -rf "$PACKAGING_DIR"
  echo "Deleted temp working directory $PACKAGING_DIR"
}

# Clean up tempdir on exit. Uncomment for debugging
trap cleanup EXIT


if [ "$1" != "" ]; then
  INSTALL_SPACE=$1
else
  INSTALL_SPACE=/opt/sasc
fi

### Packaging
echo "generating control file"
cp ${SCRIPTDIR}/sasc-control.base ${PACKAGING_DIR}/sasc-control
echo -n "Files:" >> ${PACKAGING_DIR}/sasc-control
find -L ${INSTALL_SPACE} -type f | xargs -I {} echo " {} /" >> ${PACKAGING_DIR}/sasc-control
sed -i '/^.*script (dev).tmpl.*/d' ${PACKAGING_DIR}/sasc-control
sed -i '/^.*launcher manifest.xml* /d' ${PACKAGING_DIR}/sasc-control
sed -i '/^.*darpa_logo* /d' ${PACKAGING_DIR}/sasc-control
sed -i '/^.*Screen Shot* /d' ${PACKAGING_DIR}/sasc-control

echo "Building Package with equivs"
(cd ${PACKAGING_DIR} && equivs-build ${PACKAGING_DIR}/sasc-control)

(cd ${PACKAGING_DIR} && cp *.deb $LOCAL_DIR)
