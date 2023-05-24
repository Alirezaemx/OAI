<!-- cd ci-scripts/yaml_files/5g_rfsimulator_pnf-vnf_split/ -->

In `template/.env` put VNF_ADDR, PNF_ADDR and UE0_ADDR

run `python3 configure.py -t env`

All other options can be modified after generating .env file.

BUILD_CLEAN string can be changed as required for different options at the time
of running docker compose oai-build (in generated .env file, and not in template).

To explicitly generate vnf config:
`VNF_ADDR="ipaddress" python3 configure.py -t vnf`

To explicitly generate pnf config:
`VNF_ADDR="ipaddress" PNF_ADDR="ipaddress" python3 configure.py -t pnf`


# Build
```
docker compose up oai-build
```

# VNF
```
docker compose up oai-vnf
```

# PNF
```
docker compose up oai-pnf
```

# UE
```
docker compose up oai-ue
```

# vnf, pnf, ue all together
```
docker compose up oai_vnf oai_pnf oai_ue
```

# Remove unused docker networks
```
docker network prune
```
