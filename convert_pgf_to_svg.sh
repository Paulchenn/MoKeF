#!/bin/bash

# Ordner definieren (Parameter 1, Standard: aktuelles Verzeichnis)
TARGET_DIR=${1:-.}

# Überprüfen, ob Tools installiert sind
if ! command -v pdflatex &> /dev/null; then
    echo "Fehler: 'pdflatex' wurde nicht gefunden. Bitte installiere eine LaTeX-Distribution."
    exit 1
fi

if ! command -v pdf2svg &> /dev/null; then
    echo "Fehler: 'pdf2svg' wurde nicht gefunden. Bitte installiere es mit 'brew install pdf2svg'."
    exit 1
fi

# In den Zielordner wechseln, um Sonderzeichen im absoluten Pfad zu umgehen
cd "$TARGET_DIR" || exit 1

# Output-Ordner definieren (parallel zum Zielordner, also eine Ebene höher in "svg")
OUTPUT_DIR="../svg"
mkdir -p "$OUTPUT_DIR"

# Temporäre Datei-Namen
TEX_FILE="temp_pgf_wrapper.tex"

# Alle .pgf Dateien im Zielordner durchlaufen
shopt -s nullglob
for filename in *.pgf; do
    filename_no_ext="${filename%.*}"
    
    echo "Verarbeite $filename..."

    # --- WORKAROUND FÜR TITEL UND Y-ACHSE ---
    # Matplotlib schreibt Titel und Achsen fest in die PGF-Datei. 
    # Um diese zu entfernen/ändern, kannst du sed nutzen (Kommentare entfernen und Text anpassen):
    #
    # sed -i '' '/DEIN_FALSCHER_TITEL/d' "$filename"
    # sed -i '' 's|/dein/falscher/pfad|Neue Y-Achsenbeschriftung|g' "$filename"
    # ----------------------------------------

    # 1. LaTeX-Wrapper-Dokument für das aktuelle PGF-File generieren
    # (Beinhaltet die Matplotlib-Preamble, wie in der .pgf Datei gefordert)
    cat <<EOF > "$TEX_FILE"
\documentclass{standalone}
\usepackage{pgf}
\usepackage{lmodern}
\RequirePackage[sfdefault,lf]{carlito}
\renewcommand*\familydefault{\sfdefault}
\usepackage[utf8x]{inputenc}
\usepackage[T1]{fontenc}

\def\mathdefault#1{#1}
\everymath=\expandafter{\the\everymath\displaystyle}
\makeatletter\@ifpackageloaded{underscore}{}{\usepackage[strings]{underscore}}\makeatother

\begin{document}
\input{$filename}
\end{document}
EOF

    # 2. LaTeX zu PDF kompilieren (Ausgabe wird unterdrückt, um die Konsole sauber zu halten)
    pdflatex -interaction=batchmode "$TEX_FILE" > /dev/null

    # 3. Wenn PDF erfolgreich erstellt wurde -> in SVG umwandeln
    if [ -f "temp_pgf_wrapper.pdf" ]; then
        output_svg="$OUTPUT_DIR/${filename_no_ext}.svg"
        pdf2svg "temp_pgf_wrapper.pdf" "$output_svg"
        echo " -> Erfolgreich erstellt: $output_svg"
    else
        echo " -> Fehler bei der PDF-Generierung für $filename. Bitte prüfe die Syntax der PGF-Datei."
    fi
done

# 4. Aufräumen der temporären LaTeX-Dateien
rm -f temp_pgf_wrapper.tex temp_pgf_wrapper.pdf temp_pgf_wrapper.log temp_pgf_wrapper.aux
echo "Konvertierung abgeschlossen!"
