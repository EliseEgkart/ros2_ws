#!/usr/bin/env python3
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def render_product(product_id: int, json_file: str, output_file: str | None = None) -> None:
    with Path(json_file).open("r", encoding="utf-8") as f:
        data = json.load(f)

    raw_materials = {int(k): v for k, v in data["raw_materials"].items()}
    products = {int(k): v for k, v in data["products"].items()}

    product = products[product_id]
    area = product["placement_area"]

    fig, ax = plt.subplots(figsize=(area["w"] * 0.45 + 0.4, area["h"] * 0.45 + 0.6))

    # Common 6x8 placement boundary.
    ax.add_patch(
        Rectangle((0, 0), area["w"], area["h"], fill=False, linestyle="--", linewidth=1.2, alpha=0.35)
    )

    for block in product["blocks"]:
        rid = block["raw_id"]
        mat = raw_materials[rid]
        rect = Rectangle(
            (block["x"], block["y"]),
            block["w"],
            block["h"],
            facecolor=mat["hex"],
            edgecolor="black",
            linewidth=2,
        )
        ax.add_patch(rect)
        text_color = "black" if mat["color"] == "yellow" else "white"
        ax.text(
            block["x"] + block["w"] / 2,
            block["y"] + block["h"] / 2,
            str(rid),
            ha="center",
            va="center",
            fontsize=14,
            fontweight="bold",
            color=text_color,
        )

    ax.set_xlim(-0.25, area["w"] + 0.25)
    ax.set_ylim(-0.25, area["h"] + 0.25)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(f"{product['name']} | {product_id}", fontsize=10, fontweight="bold")

    if output_file:
        fig.savefig(output_file, dpi=200, bbox_inches="tight", transparent=True)
    plt.show()


if __name__ == "__main__":
    render_product(46262, "sml_object_id_gui_assets_6x8.json", "product_46262_6x8.png")
