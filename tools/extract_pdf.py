import sys
import pathlib

def try_pypdf(path):
    try:
        from pypdf import PdfReader
    except Exception:
        try:
            from PyPDF2 import PdfReader
        except Exception:
            return None
    reader = PdfReader(path)
    return "\n".join((page.extract_text() or "") for page in reader.pages)

def try_fitz(path):
    try:
        import fitz
    except Exception:
        return None
    doc = fitz.open(path)
    return "\n".join(page.get_text() for page in doc)

def try_pdfminer(path):
    try:
        from pdfminer.high_level import extract_text
    except Exception:
        return None
    return extract_text(path)

def main():
    src = pathlib.Path(sys.argv[1])
    out = pathlib.Path(sys.argv[2])
    for name, fn in (("pypdf", try_pypdf), ("fitz", try_fitz), ("pdfminer", try_pdfminer)):
        text = fn(str(src))
        if text and text.strip():
            out.write_text(text, encoding="utf-8")
            print(f"OK via {name}: {len(text)} chars -> {out}")
            return
        print(f"{name}: unavailable or empty")
    print("FAILED: no usable PDF library")

if __name__ == "__main__":
    main()
