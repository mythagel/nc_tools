-- Complete general configuration for nc tools
-- searched for in cwd of tool, or specified with config file 
-- (cwd more convenient - expected that tools will be executed from project root)
-- not all configuration used by all tools BUT ONE config file for all tools with different
-- sections set.

-- Machine configuration
machine = {
	type = "mill",
	spindle = {"100-1000", "2000-6000"}
}

-- Tool table - tool definitions could be imported from other files...
tool_table = {
    [3] = {
        name = "8mm carbide end mill",
        length = 50,
        diameter = 8,
        flute_length = 20,
        shank_diameter = 8
    }
}
