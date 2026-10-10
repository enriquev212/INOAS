"""Compose the README overview from unchanged vector panels in the paper."""

import argparse
from copy import copy
from io import BytesIO
from pathlib import Path

from pypdf import PdfReader, PdfWriter, Transformation
from pypdf.generic import RectangleObject
from reportlab.pdfgen import canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manuscript", type=Path)
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).with_name("evaluation-overview.pdf"),
    )
    args = parser.parse_args()
    if args.manuscript.resolve() == args.output.resolve():
        parser.error("The output must not replace the source manuscript.")

    source = PdfReader(args.manuscript)
    if len(source.pages) < 15:
        parser.error("Expected the final 20-page manuscript, with figures on pages 14 and 15.")

    background = BytesIO()
    drawing = canvas.Canvas(background, pagesize=(640, 330))
    drawing.setFont("Helvetica-Bold", 15)
    drawing.drawString(24, 308, "Navigation precision")
    drawing.drawString(290, 308, "Maneuver efficiency")

    drawing.setFont("Helvetica", 10)
    for x, color, label in (
        (295, "#999999", "Full GNSS"),
        (400, "#24549F", "Fixed-Time"),
        (510, "#C5352A", "Reactive"),
    ):
        drawing.setFillColor(color)
        if label == "Reactive":
            marker = drawing.beginPath()
            marker.moveTo(x, 290)
            marker.lineTo(x + 4, 286)
            marker.lineTo(x, 282)
            marker.lineTo(x - 4, 286)
            marker.close()
            drawing.drawPath(marker, stroke=0, fill=1)
        else:
            drawing.circle(x, 286, 3, stroke=0, fill=1)
        drawing.setFillColor("#222222")
        drawing.drawString(x + 8, 282, label)
    drawing.setFont("Helvetica", 9)
    drawing.drawString(290, 267, "Right panel: filled = adaptive; open = constant 295 m")
    drawing.showPage()
    drawing.save()
    background.seek(0)

    writer = PdfWriter()
    output_page = writer.add_page(PdfReader(background).pages[0])
    # Rectangles are top-left PDF coordinates converted from the inspected 300 dpi crops.
    panels = (
        (13, (397.44, 99.55, 568.08, 324.24), 1.17, (24, 25)),
        (14, (390.00, 222.40, 570.00, 358.80), 1.75, (290, 24)),
    )
    for index, (left, top, right, bottom), scale, (x, y) in panels:
        panel = copy(source.pages[index])
        if tuple(float(v) for v in panel.mediabox) != (0, 0, 612, 792):
            parser.error("Crop coordinates require the final Letter-size manuscript.")
        lower = 792 - bottom
        panel.cropbox = RectangleObject((left, lower, right, 792 - top))
        output_page.merge_transformed_page(
            panel,
            Transformation((scale, 0, 0, scale, x - scale * left, y - scale * lower)),
        )

    writer.add_metadata({
        "/Title": "INOAS: navigation precision and maneuver efficiency",
        "/Subject": "Unchanged data panels from paper Figs. 4(d) and 5(c), with presentation headings and legend.",
        "/Creator": "render_evaluation_overview.py",
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as output:
        writer.write(output)


if __name__ == "__main__":
    main()
