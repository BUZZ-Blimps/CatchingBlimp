#pragma once

enum object {
    balloon,
    blimpB,
    blimpR,
    goalO,
    goalY,
    first = balloon,
    last = goalY
};

enum autoState{
	searching,
	approach,
	catching,
	caught,
	goalSearch,
	approachGoal,
	scoringStart,
	shooting,
	scored
};

enum blimpType{
	blue,
	red
};

enum goalType{
	orange,
	yellow
};