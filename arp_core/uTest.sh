TEST_OUTPUT_DIR=`rospack find arp_core`/test/output
mkdir -p $TEST_OUTPUT_DIR

`rospack find arp_core`/bin/uTest_Core_Math.ard  --output_format=XML --log_level=all --report_level=no > $TEST_OUTPUT_DIR/uTest_Core_Math.xml

