TEST_OUTPUT_DIR=`rospack find arp_rlu`/test/output
mkdir -p $TEST_OUTPUT_DIR

`rospack find arp_rlu`/bin/uTest_KFL.ard  --output_format=XML --log_level=all --report_level=no > $TEST_OUTPUT_DIR/uTest_KFL.xml
`rospack find arp_rlu`/bin/uTest_LSL.ard  --output_format=XML --log_level=all --report_level=no > $TEST_OUTPUT_DIR/uTest_LSL.xml


