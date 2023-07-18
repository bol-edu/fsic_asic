#!/usr/bin/perl

# --------------------------------------------------------
# Change Verilog 1995 to System Verilog friendly Simulator
# add wire to input/output port


use strict;
use warnings;

my $file = $ARGV[0];
my $line;
open (FH, '<', $file) or die $!;

while (<FH>) {
 $line = $_;

 if( $line =~ /^\s*input\s*([\d|\w]+)([,|;])/) {
   print "input wire $1"."$2\n";
   next;
 } 

 if( $line =~ /^\s*input\s*([\d|\w]+)$/) {
   print "input wire $1\n";
   next;
 } 

 if( $line =~ /^\s*input\s*(\[.*\])\s*([\d|\w]+)([,|;])/) {
   print "input wire $1 $2"."$3\n";
   next;
 } 

 if( $line =~ /^\s*input\s*(\[.*\])\s*([\d|\w]+)$/) {
   print "input wire $1 $2\n";
   next;
 } 

 if( $line =~ /^\s*output\s*([\d|\w]+)([,|;])/) {
   print "output wire $1"."$2\n";
   next;
 } 

 if( $line =~ /^\s*output\s*([\d|\w]+)\s*\/\/(.*)$/) {
   print "output wire $1 \/\/ $2\n";
   next;
 }

 if( $line =~ /^\s*output\s*([\d|\w]+)$/) {
   print "output wire $1\n";
   next;
 }

 if( $line =~ /^\s*output\s*(\[.*\])\s*([\d|\w]+)([,|;])/) {
   print "output wire $1 $2"."$3\n";
   next;
 } 

 if( $line =~ /^\s*output\s*(\[.*\])\s*([\d|\w]+)\s*\/\/(.*)$/) {
   print "output wire $1 $2"." \/\/ $3\n";
   next;
 } 

 if( $line =~ /^\s*output\s*(\[.*\])\s*([\d|\w]+)$/) {
   print "output wire $1 $2\n";
   next;
 } 

 if( $line =~ /^\s*inout\s*([\d|\w]+)([,|;])/) {
   print "inout wire $1"."$2\n";
   next;
 }

 if( $line =~ /^\s*inout\s*([\d|\w]+)$/) {
   print "inout wire $1\n";
   next;
 }

 if( $line =~ /^\s*inout\s*(\[.*\])\s*([\d|\w]+)([,|;])/) {
   print "inout wire $1 $2"."$3\n";
   next;
 }

 if( $line =~ /^\s*inout\s*(\[.*\])\s*([\d|\w]+)$/) {
   print "inout wire $1 $2\n";
   next;
 }

 print $line;

}

close(FH);
