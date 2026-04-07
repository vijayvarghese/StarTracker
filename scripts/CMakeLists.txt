
add_executable(Bin_Gen
script_gen_binLookup.cpp
)

add_executable(Lookup_Gen
gen_lookup.c
)

target_link_libraries(Lookup_Gen
m
)