#!/usr/bin/env python3
import argparse
import os
import signal
from pathlib import Path

def interrupt(signum, frame):
  raise Exception("")


def stdinWait(text, default, time, timeoutDisplay = None, **kwargs):
  signal.signal(signal.SIGALRM, interrupt)
  signal.alarm(time) # sets timeout
  global timeout
  try:
    inp = input(text)
    signal.alarm(0)
    timeout = False
  except (KeyboardInterrupt):
    printInterrupt = kwargs.get("printInterrupt", True)
    if printInterrupt:
      print ("Keyboard interrupt")
    timeout = True # Do this so you don't mistakenly get input when there is none
    inp = default
  except:
    timeout = True
    if not timeoutDisplay is None:
      print (f"{timeoutDisplay}")
    signal.alarm(0)
    inp = default
  return inp


def get_active_branch_name(mainFolder):
  head_dir = os.path.join(mainFolder, 'openairinterface5g', '.git', 'HEAD')
  with open(head_dir,"r") as f:
    content = f.read().splitlines()
  for line in content:
    if line[0:4] == "ref:":
      return line.partition("refs/heads/")[2]


parser = argparse.ArgumentParser()

parser.add_argument("-t", "--type",
                    action="store",
                    default=None,
                    required=True,
                    help="env/gnb/ue")

args = parser.parse_args()

py_file_path = os.path.dirname(os.path.realpath(__file__))

print('\n\n')

if args.type == "env":
  conf_file_name = '.env'
  template_file_path = os.path.join(py_file_path, 'template', conf_file_name)
  final_conf_dir = py_file_path
  final_conf_path = os.path.join(final_conf_dir, conf_file_name)

  with open(template_file_path,"r") as f:
    conf = f.read()
    if os.environ.get('USER') is not None:
      print(f"Hi, {os.environ['USER'].split('.')[0]}")
      print(f".env file path: {final_conf_path}")
      print(f"Root folder is: {py_file_path.split('openairinterface5g')[0]}")
      conf = conf.replace("{{DEV}}", os.environ['USER'].split('.')[0])
      conf = conf.replace("{{USERID}}", str(os.getuid()))
      conf = conf.replace("{{ROOT_VOL}}", py_file_path.split('openairinterface5g')[0][:-1])

      cpn = os.environ['USER'].split('.')[0] \
          + '_' \
          + Path(py_file_path.split('openairinterface5g')[0][:-1]).parts[-1] \
          + '_' \
          + get_active_branch_name(py_file_path.split('openairinterface5g')[0][:-1])[:10] # consider first 10 chars
      timeout = None
      timeoutLimit = 5


      inp = stdinWait(f"\nEnter Y/y within {timeoutLimit} seconds to create your own COMPOSE_PROJECT_NAME:{cpn} (present) and press <Enter>...: ", cpn, timeoutLimit, f"\nOkay, nothing entered.")
      if not timeout:
        print (f"You entered {inp}")
        if (inp=='Y' or inp=='y'):
          cpn_inp = input(f"\nEnter your COMPOSE_PROJECT_NAME and press Enter: ")
          if cpn_inp.strip() != '':
            cpn = cpn_inp.strip()
          else:
            print("COMPOSE_PROJECT_NAME can not be blank.")
          print (f"\nWill be using COMPOSE_PROJECT_NAME={cpn}")
      else:
        print (f"\nWill be using COMPOSE_PROJECT_NAME={cpn}")

      conf = conf.replace("{{PROJECT_NAME}}", cpn)

      with open(final_conf_path, "w") as conf_file:
        conf_file.write(conf)
    else:
      print("USER is not available in environment variable")

  print("\n.env file generated.")
elif args.type == "vnf":
  conf_file_name = 'sCont.vnf.sa.band78.106prb.rfsim.conf'
  template_file_path = os.path.join(py_file_path, 'template', conf_file_name)
  final_conf_dir = os.path.join(py_file_path, 'conf')
  final_conf_path = os.path.join(final_conf_dir, conf_file_name)
  Path(final_conf_dir).mkdir(parents=True, exist_ok=True)

  with open(template_file_path,"r") as f:
    conf = f.read()
    if os.environ.get('VNF_ADDR') is not None:
      print(f"VNF_ADDR = {os.environ['VNF_ADDR']}")
      print(f"Conf file path: {final_conf_path}")
      conf = conf.replace("{{VNF_ADDR}}", os.environ['VNF_ADDR'])
      with open(final_conf_path, "w") as conf_file:
        conf_file.write(conf)
    else:
      print("VNF_ADDR is not available in environment variable")
elif args.type == "pnf":
  conf_file_name = 'sCont.pnf.sa.band78.rfsim.conf'
  template_file_path = os.path.join(py_file_path, 'template', conf_file_name)
  final_conf_dir = os.path.join(py_file_path, 'conf')
  final_conf_path = os.path.join(final_conf_dir, conf_file_name)
  Path(final_conf_dir).mkdir(parents=True, exist_ok=True)

  print(f"Conf file path: {final_conf_path}")
  with open(template_file_path,"r") as f:
    conf = f.read()
    if os.environ.get('VNF_ADDR') is not None:
      print(f"VNF_ADDR = {os.environ['VNF_ADDR']}")
      conf = conf.replace("{{VNF_ADDR}}", os.environ['VNF_ADDR'])
    else:
      print("VNF_ADDR is not available in environment variable")

    if os.environ.get('PNF_ADDR') is not None:
      print(f"PNF_ADDR = {os.environ['PNF_ADDR']}")
      conf = conf.replace("{{PNF_ADDR}}", os.environ['PNF_ADDR'])
    else:
      print("PNF_ADDR is not available in environment variable")
    with open(final_conf_path, "w") as conf_file:
      conf_file.write(conf)
elif args.type == "ue":
  conf_file_name = 'sCont.ue.conf'
  template_file_path = os.path.join(py_file_path, 'template', conf_file_name)
  final_conf_dir = os.path.join(py_file_path, 'conf')
  final_conf_path = os.path.join(final_conf_dir, conf_file_name)
  Path(final_conf_dir).mkdir(parents=True, exist_ok=True)

  with open(template_file_path,"r") as f:
    conf = f.read()
    with open(final_conf_path, "w") as conf_file:
      conf_file.write(conf)
else:
  print(f"Wrong argument type")
