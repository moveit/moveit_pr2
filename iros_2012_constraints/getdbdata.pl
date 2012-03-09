#!/usr/bin/perl -w

my @C = ([1000, 0], [1000, 20], [1000, 100],
	 [5000, 0], [5000, 20], [5000, 100],
	 [10000, 0], [10000, 20], [10000, 100]);

my %r=();

foreach my $i(1..3)
{
    foreach my $c(@C)
    {
	my ($s, $e) = @$c;
	print $s, " ", $e, "\n";
	`rm -f /home/isucan/c/*`;
	next unless ($s > 0);
	my @out = `bin/exp$i __ns:=ompl_planning $s $e`;
	my $out = join('', @out);
	$out =~ /Spend ([\d\.]+) seconds constructing/;
	my $time = $1;
	$out =~ /Added (\d+) connexions/;
	my $edges = $1;
	$edges = 0 if ($e == 0);
	$out =~ /Constrained sampling rate: ([\d\.]+)/;
	my $srate = $1 * 100.0;
	my @outsz = `ls -l /home/isucan/c/*.ompldb`;
	my $outsz = join('', @outsz);
	$outsz =~ /isucan (\d+)/;
	my $mbytes = $1 / 1048576.0;
	$r{"$s/$e"}{'time'} = sprintf("%.1f",  $time);
	$r{"$s/$e"}{'size'} = sprintf("%.1f",  $mbytes);
	$r{"$s/$e"}{'success'} = sprintf("%.0f",  $srate);
	$r{"$s/$e"}{'states'} = $s;
	$r{"$s/$e"}{'edges'} = $edges;
	my $ddd = "/home/isucan/projects/moveit_pr2/iros_2012_constraints/exp/exp$i/db_$s".'_'.$e;
	`mkdir -p $ddd`;
	open(FOUT, ">/home/isucan/projects/moveit_pr2/iros_2012_constraints/exp/exp$i/db_".$s."_$e/README");
	print FOUT "T: $s/$e\n";
	print FOUT $r{"$s/$e"}{'time'}, "\n";
	print FOUT $r{"$s/$e"}{'size'}, "\n";
	print FOUT $r{"$s/$e"}{'success'}, "\n";
	print FOUT $r{"$s/$e"}{'states'}, "\n";
	print FOUT $r{"$s/$e"}{'edges'}, "\n";
	close(FOUT);
    }
    print "exp$i\n";
    my @k = sort keys %r;
    foreach my $k(@k)
    {
	print $k, ' & ', $r{$k}{'time'}, ' & ', $r{$k}{'size'}, "\\\\\n";
    }
    
    %r = ();    
}

