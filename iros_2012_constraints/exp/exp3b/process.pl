#!/usr/bin/perl
 
# Import needed Modules
use DBI;
use strict;

my @dirs = `ls -1`;
chomp(@dirs);
@dirs = grep(-d $_, @dirs);
my %res;
my %res2;

foreach my $d(@dirs)
{
    print "$d\n";
    my @contextL = grep($_ =~ /T: /, `cat  $d/README`);
    my $context = $contextL[0];
    $context =~ s/^T:\s*//; $context =~ s/\s+$//;
    print "$context\n";
    my $dbh = DBI->connect("dbi:SQLite:dbname=$d/benchmark.db","","");
    my $tl = $dbh->prepare("SELECT name FROM sqlite_master WHERE type='table'");
    $tl->execute();
    my @tbl = ();
    while (my $tbl = $tl->fetchrow_arrayref)
    {
	push(@tbl, $$tbl[0]) if $$tbl[0] =~ /planner_geometric/;
    }
    
    foreach my $t(@tbl)
    {
	my ($solved) = $dbh->selectrow_array("SELECT AVG(solved) FROM `$t`");
	my ($time, $length) = $dbh->selectrow_array("SELECT AVG(time), AVG(solution_length) FROM `$t` WHERE solved = 1");
	my $alg = $t; $alg =~ s/planner_geometric_//; $alg =~ s/1//;
	print "$alg: $solved, $time, $length\n";
	$res{$alg}{$context} = [$solved, $time, $length];
	$res2{$context}{$alg} = [$solved, $time, $length];
    }
}
my @alg = qw/RRTConnect LBKPIECE SBL RRT KPIECE/;#sort keys %res;
my @contexts = qw/0.5 0.6 0.8 0.9/;
print '\\begin{tabular}{l|', 'c' x @alg, '}', "\n";
print "\\toprule\n";
foreach my $a(@alg) {
    print " & $a"; }
print "\\\\\n";
print "\\midrule\n";
foreach my $c(@contexts)
{
    print $c;
    foreach my $a(@alg)
    {
	my ($s, $t, $l) = @{$res2{$c}{$a} || [-1, -1, -1]};
	if ($s < 0.0)
	{
	    print " & N/A";
	}
	else
	{
	    $s = sprintf("%.0f", $s * 100.0);
	    $t = sprintf("%.2f", $t);
	    print " & $t / $s\\%";
	}
    }
    print "\\\\\n";
}  
print "\\bottomrule\n";
print '\end{tabular}', "\n";

