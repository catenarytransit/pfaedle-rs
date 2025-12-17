use css_color::Srgb;
use std::str::FromStr;

pub fn parse_color(input: &str) -> Option<(String, String)> {
    let color = Srgb::from_str(input).ok()?;

    let hex_bg = format!(
        "{:02X}{:02X}{:02X}",
        (color.red * 255.0).round() as u8,
        (color.green * 255.0).round() as u8,
        (color.blue * 255.0).round() as u8
    );

    // Calculate relative luminance according to WCAG 2.0 standards.
    // This requires linearizing the sRGB channels (gamma expansion) before applying coefficients.
    // Coefficients: 0.2126 (R), 0.7152 (G), 0.0722 (B).
    let linear = |c: f32| -> f32 {
        if c <= 0.03928 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        }
    };

    let lum = 0.2126 * linear(color.red)
        + 0.7152 * linear(color.green)
        + 0.0722 * linear(color.blue);

    // Determine foreground color based on the highest contrast ratio.
    // Ratio formula: (L1 + 0.05) / (L2 + 0.05), where L is relative luminance.
    let contrast_white = 1.05 / (lum + 0.05);
    let contrast_black = (lum + 0.05) / 0.05;

    let hex_fg = if contrast_white > contrast_black {
        "FFFFFF".to_string()
    } else {
        "000000".to_string()
    };

    Some((hex_bg, hex_fg))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_hex() {
        let (bg, fg) = parse_color("#FF0000").unwrap();
        assert_eq!(bg, "FF0000");
        // Pure Red (#F00) is visually bright but mathematically dark (Lum ~0.21).
        // Contrast vs Black: ~5.25:1. Contrast vs White: ~4.0:1.
        // Therefore, Black text is the strict WCAG preference, despite common design patterns.
        assert_eq!(fg, "000000");

        let (bg, fg) = parse_color("white").unwrap();
        assert_eq!(bg, "FFFFFF");
        assert_eq!(fg, "000000");

        let (bg, fg) = parse_color("black").unwrap();
        assert_eq!(bg, "000000");
        assert_eq!(fg, "FFFFFF");
    }
}
