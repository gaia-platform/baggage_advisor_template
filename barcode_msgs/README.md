# barcode_msgs

This package provides messages for generic barcodes and scanning barcodes from images.

## Messages (.msg)
* [Barcode](msg/Barcode.msg): Represents a single barcode, with its barcode type, data, and geometric points representing its location in an image.
* [ScanResult](msg/ScanResult.msg): Contains an array of barcodes and a reference frame and timestamp. A single Scan Result represents all the barcodes decoded from a single image/video frame.
