#!/usr/bin/perl -w

my @C = ([5000, 0], [5000, 20], [5000, 100], [10000, 0], [10000, 20], [10000, 100], [1000, 0], [1000, 20], [1000, 100]);

foreach my $i(3)
{
    foreach my $c(@C)
    {
	my ($s, $e) = @$c;
	print $s, " ", $e, "\n";
	`rm -f /home/isucan/c/*`;
	if ($s > 0)
	{
	    `bin/exp$i __ns:=ompl_planning $s $e`;
	}
	my $pid = fork();
	if ($pid == 0)
	{
	    exec("/home/isucan/projects/moveit_ros/ompl_interface_ros/bin/ompl_planner");
	    die;
	} else {
	    `bin/exp$i __ns:=ompl_planning`;
	    kill 9, $pid;
	    my $dir = "exp/exp$i/db_".$s."_".$e;    
	    `mkdir -p /home/isucan/projects/moveit_pr2/iros_2012_constraints/$dir`;
	    `rm -f /home/isucan/projects/moveit_pr2/iros_2012_constraints/$dir/ompl_*`;
	    `mv ompl_* /home/isucan/projects/moveit_pr2/iros_2012_constraints/$dir`;
	    `cd /home/isucan/projects/moveit_pr2/iros_2012_constraints/$dir && /home/isucan/projects/moveit_pr2/iros_2012_constraints/exp/ala.py ompl_* -p p.pdf && cd -`;
	}
    }
}
