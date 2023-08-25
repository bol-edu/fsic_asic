
assign mprj_io[3] = 1;  // Force CSB high.
assign mprj_io[0] = 0;  // Disable Debug Mode


initial begin
  wait(checkbits == 16'hAB40);
  $display("LA Test 1 started");
  wait(checkbits == 16'hAB41);
  wait(checkbits == 16'hAB51);
  $display("LA Test 2 passed");
  #5000;
  $finish;
end

