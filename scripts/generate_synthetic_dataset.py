#!/usr/bin/env python3
"""
Generate synthetic traffic sign dataset for YOLO11 training
Creates colored rectangular signs with random positions, sizes, and backgrounds

Usage:
    python3 generate_synthetic_dataset.py --num-images 1000 --output-dir ~/teamsteelbot_ws/datasets/traffic_signs
"""

import cv2
import numpy as np
import argparse
from pathlib import Path
import random


def generate_sign_image(sign_color, background_type='random'):
    """
    Generate synthetic sign image with label

    Args:
        sign_color: 'red', 'green', or 'blue'
        background_type: 'random', 'gray', 'textured'

    Returns:
        img: 640x480 BGR image
        label: YOLO format label string
    """
    img_h, img_w = 480, 640

    # Generate background
    if background_type == 'random':
        # Random noise background
        bg_brightness = random.randint(80, 160)
        img = np.random.randint(
            max(0, bg_brightness - 30),
            min(255, bg_brightness + 30),
            (img_h, img_w, 3),
            dtype=np.uint8
        )
    elif background_type == 'gray':
        # Solid gray
        gray_val = random.randint(100, 150)
        img = np.full((img_h, img_w, 3), gray_val, dtype=np.uint8)
    else:  # textured
        # Perlin-like texture (simple version)
        img = np.random.randint(100, 150, (img_h, img_w, 3), dtype=np.uint8)
        img = cv2.GaussianBlur(img, (51, 51), 0)

    # Random sign dimensions (more variation for robustness)
    sign_w = random.randint(60, 200)
    sign_h = random.randint(80, 250)

    # Random position (ensure sign is fully visible)
    margin = 20
    x = random.randint(margin, img_w - sign_w - margin)
    y = random.randint(margin, img_h - sign_h - margin)

    # Color map (BGR format)
    colors = {
        'red': (0, 0, 255),
        'green': (0, 255, 0),
        'blue': (255, 0, 0),
    }

    # Add slight color variation for realism
    base_color = np.array(colors[sign_color])
    color_variation = np.random.randint(-20, 20, 3)
    color = np.clip(base_color + color_variation, 0, 255).astype(int).tolist()

    # Draw sign (filled rectangle)
    cv2.rectangle(img, (x, y), (x + sign_w, y + sign_h), color, -1)

    # Add white border (with random thickness)
    border_thickness = random.randint(2, 5)
    cv2.rectangle(img, (x, y), (x + sign_w, y + sign_h),
                 (255, 255, 255), border_thickness)

    # Optional: Add inner rectangle (like real traffic signs)
    if random.random() > 0.5:
        inner_margin = random.randint(8, 15)
        cv2.rectangle(img,
                     (x + inner_margin, y + inner_margin),
                     (x + sign_w - inner_margin, y + sign_h - inner_margin),
                     (255, 255, 255),
                     border_thickness)

    # Add noise and blur for realism
    if random.random() > 0.3:
        noise = np.random.randint(-15, 15, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    if random.random() > 0.5:
        blur_size = random.choice([3, 5])
        img = cv2.GaussianBlur(img, (blur_size, blur_size), 0)

    # Optional: Add rotation (simulates viewing angle)
    if random.random() > 0.7:
        angle = random.uniform(-15, 15)
        center = (x + sign_w // 2, y + sign_h // 2)
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        img = cv2.warpAffine(img, M, (img_w, img_h))

    # Create YOLO label (normalized coordinates)
    class_id = {'red': 0, 'green': 1, 'blue': 2}[sign_color]
    x_center = (x + sign_w / 2) / img_w
    y_center = (y + sign_h / 2) / img_h
    width = sign_w / img_w
    height = sign_h / img_h

    label = f'{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}'

    return img, label


def generate_multi_sign_image():
    """Generate image with multiple signs (harder examples)"""
    img_h, img_w = 480, 640

    # Background
    bg_brightness = random.randint(100, 150)
    img = np.random.randint(
        max(0, bg_brightness - 30),
        min(255, bg_brightness + 30),
        (img_h, img_w, 3),
        dtype=np.uint8
    )

    labels = []
    num_signs = random.randint(2, 3)

    for _ in range(num_signs):
        sign_color = random.choice(['red', 'green', 'blue'])

        # Smaller signs for multi-sign images
        sign_w = random.randint(50, 120)
        sign_h = random.randint(70, 150)

        # Random position
        x = random.randint(10, img_w - sign_w - 10)
        y = random.randint(10, img_h - sign_h - 10)

        # Draw sign
        colors = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
        }
        color = colors[sign_color]

        cv2.rectangle(img, (x, y), (x + sign_w, y + sign_h), color, -1)
        cv2.rectangle(img, (x, y), (x + sign_w, y + sign_h), (255, 255, 255), 3)

        # Create label
        class_id = {'red': 0, 'green': 1, 'blue': 2}[sign_color]
        x_center = (x + sign_w / 2) / img_w
        y_center = (y + sign_h / 2) / img_h
        width = sign_w / img_w
        height = sign_h / img_h

        labels.append(f'{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}')

    return img, '\n'.join(labels)


def main():
    parser = argparse.ArgumentParser(description='Generate synthetic traffic sign dataset')
    parser.add_argument('--num-images', type=int, default=1000,
                       help='Number of images to generate')
    parser.add_argument('--output-dir', type=str,
                       default='~/teamsteelbot_ws/datasets/traffic_signs',
                       help='Output directory')
    parser.add_argument('--train-split', type=float, default=0.8,
                       help='Train/val split ratio (default: 0.8)')
    parser.add_argument('--multi-sign-ratio', type=float, default=0.2,
                       help='Ratio of images with multiple signs (default: 0.2)')

    args = parser.parse_args()

    # Expand path
    output_dir = Path(args.output_dir).expanduser()

    # Create directories
    train_img_dir = output_dir / 'images' / 'train'
    train_lbl_dir = output_dir / 'labels' / 'train'
    val_img_dir = output_dir / 'images' / 'val'
    val_lbl_dir = output_dir / 'labels' / 'val'

    for d in [train_img_dir, train_lbl_dir, val_img_dir, val_lbl_dir]:
        d.mkdir(parents=True, exist_ok=True)

    print(f'Generating {args.num_images} synthetic images...')
    print(f'Output directory: {output_dir}')
    print(f'Train/val split: {args.train_split:.0%} / {1-args.train_split:.0%}')
    print(f'Multi-sign images: {args.multi_sign_ratio:.0%}')
    print()

    # Generate images
    for img_id in range(args.num_images):
        # Determine if this is a training or validation image
        is_train = (img_id / args.num_images) < args.train_split

        img_dir = train_img_dir if is_train else val_img_dir
        lbl_dir = train_lbl_dir if is_train else val_lbl_dir

        # Determine if multi-sign
        is_multi = random.random() < args.multi_sign_ratio

        if is_multi:
            img, label = generate_multi_sign_image()
        else:
            # Equal distribution of colors
            sign_color = ['red', 'green', 'blue'][img_id % 3]
            background = random.choice(['random', 'gray', 'textured'])
            img, label = generate_sign_image(sign_color, background)

        # Save image
        img_path = img_dir / f'{img_id:05d}.jpg'
        cv2.imwrite(str(img_path), img)

        # Save label
        lbl_path = lbl_dir / f'{img_id:05d}.txt'
        with open(lbl_path, 'w') as f:
            f.write(label)

        if (img_id + 1) % 100 == 0:
            print(f'Generated {img_id + 1}/{args.num_images} images...')

    print()
    print('✅ Dataset generation complete!')
    print()
    print('Summary:')
    print(f'  Train images: {len(list(train_img_dir.glob("*.jpg")))}')
    print(f'  Val images:   {len(list(val_img_dir.glob("*.jpg")))}')
    print()
    print('Next steps:')
    print(f'  1. Create data.yaml in {output_dir}')
    print('  2. Train YOLO11: yolo detect train data=data.yaml model=yolo11n.pt')
    print()
    print('Example data.yaml:')
    print(f'''
path: {output_dir}
train: images/train
val: images/val

nc: 3
names: ['red_sign', 'green_sign', 'blue_sign']
''')


if __name__ == '__main__':
    main()
