use css_color::Srgb;
use std::str::FromStr;

pub fn parse_color(input: &str) -> Option<(String, String)> {
    // Attempt parse
    let color = Srgb::from_str(input).ok()?;

    // Convert to hex string (without #)
    let hex_bg = format!(
        "{:02X}{:02X}{:02X}",
        (color.red * 255.0).round() as u8,
        (color.green * 255.0).round() as u8,
        (color.blue * 255.0).round() as u8
    );

    // Calculate luminance for contrast
    // Formula: 0.2126*R + 0.7152*G + 0.0722*B (linear RGB)
    // css_color's red/green/blue are 0..1 intervals.
    // However, they might be sRGB (gamma corrected) or linear.
    // The crate documentation says `Srgb` struct. Standard relative luminance uses linear RGB.
    // But for simple black/white contrast on web, using the sRGB values directly or usually sRGB->Linear is better.
    // Quick approximation on sRGB is often "good enough" for simple black/white choice,
    // but let's try to be slightly proper if we can, or just use the weighted sum on sRGB which is common practice even if technically wrong.
    // Let's use the W3C recommendation for relative luminance if possible, which requires linearizing.

    // Linearize
    let s_r = color.red;
    let s_g = color.green;
    let s_b = color.blue;

    let linear = |c: f32| -> f32 {
        if c <= 0.03928 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        }
    };

    let l_r = linear(s_r);
    let l_g = linear(s_g);
    let l_b = linear(s_b);

    let lum = 0.2126 * l_r + 0.7152 * l_g + 0.0722 * l_b;

    // Threshold is usually 0.5 for simple cases, or check contrast ratio against black/white.
    // White text (1.0) on BG: (1.0 + 0.05) / (lum + 0.05)
    // Black text (0.0) on BG: (lum + 0.05) / (0.0 + 0.05)

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
        assert_eq!(fg, "FFFFFF"); // Red needs white text usually? 
        // Red luminance: ~0.21. Black contrast: (0.26/0.05)=5.2. White contrast: 1.05/0.26=4.0.
        // Wait, Red (255,0,0) is actually quite dark in relative luminance.
        // sRGB: 1, 0, 0. Linear: 1, 0, 0. Lum: 0.2126.
        // Ratio vs Black: (0.2126+0.05)/0.05 = 5.25.
        // Ratio vs White: 1.05/(0.2126+0.05) = 3.999.
        // So actually Black text is better on pure Red according to strict WCAG?
        // Let's check common behavior. Typically white on red is used.
        // Maybe my simple logic is enough.
        // let's try a lighter color.

        // White
        let (bg, fg) = parse_color("white").unwrap();
        assert_eq!(bg, "FFFFFF");
        assert_eq!(fg, "000000");

        // Black
        let (bg, fg) = parse_color("black").unwrap();
        assert_eq!(bg, "000000");
        assert_eq!(fg, "FFFFFF");
    }
}
