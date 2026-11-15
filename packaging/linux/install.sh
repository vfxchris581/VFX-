#!/bin/sh
# Install HD for the current user (default) or system-wide:
#   ./install.sh                 -> ~/.local
#   sudo ./install.sh /usr/local -> /usr/local
# Remove again with the uninstall.sh that this script installs next to the program.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
PREFIX=${1:-${PREFIX:-$HOME/.local}}
LIB="$PREFIX/lib/hd"
mkdir -p "$LIB" "$PREFIX/bin" "$PREFIX/share/applications" "$PREFIX/share/icons/hicolor/256x256/apps" "$PREFIX/share/mime/packages"

rm -rf "$LIB/examples"
cp "$HERE/HD" "$HERE/hdc" "$HERE/README.md" "$LIB/"
cp -R "$HERE/examples" "$LIB/examples"
chmod 755 "$LIB/HD" "$LIB/hdc"
ln -sf "$LIB/HD" "$PREFIX/bin/HD"
ln -sf "$LIB/hdc" "$PREFIX/bin/hdc"

sed "s|^Exec=HD|Exec=$LIB/HD|" "$HERE/hd.desktop" > "$PREFIX/share/applications/hd.desktop"
cp "$HERE/hd.png" "$PREFIX/share/icons/hicolor/256x256/apps/hd.png"
cp "$HERE/hd-mime.xml" "$PREFIX/share/mime/packages/hd.xml"

cat > "$LIB/uninstall.sh" <<UNINSTALL
#!/bin/sh
set -eu
rm -f "$PREFIX/bin/HD" "$PREFIX/bin/hdc" "$PREFIX/share/applications/hd.desktop" \\
      "$PREFIX/share/icons/hicolor/256x256/apps/hd.png" "$PREFIX/share/mime/packages/hd.xml"
rm -rf "$LIB"
command -v update-mime-database >/dev/null 2>&1 && update-mime-database "$PREFIX/share/mime" >/dev/null 2>&1 || true
command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database "$PREFIX/share/applications" >/dev/null 2>&1 || true
echo "HD removed from $PREFIX"
UNINSTALL
chmod 755 "$LIB/uninstall.sh"

command -v update-mime-database >/dev/null 2>&1 && update-mime-database "$PREFIX/share/mime" >/dev/null 2>&1 || true
command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database "$PREFIX/share/applications" >/dev/null 2>&1 || true
command -v gtk-update-icon-cache >/dev/null 2>&1 && gtk-update-icon-cache -q "$PREFIX/share/icons/hicolor" >/dev/null 2>&1 || true

echo "HD $("$LIB/hdc" --version | cut -d' ' -f2) installed to $LIB"
case ":$PATH:" in *":$PREFIX/bin:"*) ;; *) echo "Add $PREFIX/bin to your PATH to run HD and hdc from a terminal." ;; esac
echo "Uninstall with: $LIB/uninstall.sh"
