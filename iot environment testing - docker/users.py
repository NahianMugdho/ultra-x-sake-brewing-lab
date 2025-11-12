#!/usr/bin/env python3
"""
generate_users.py

Generate a CSV file of users with simple passwords.
By default it creates 1000 users (including the admin row).

Usage:
    python generate_users.py
    python generate_users.py --total 500 --outfile my_users.csv
"""

import csv
import argparse
import sys

def generate_users_csv(total=1000, outfile="users.csv", include_admin=True):
    """
    Generate a CSV file with `total` users. If include_admin is True,
    the first row after the header will be the admin user with password "123".
    The remaining users will be user1..userN with passwords pass1..passN.

    total: total number of user rows (including admin if include_admin=True).
    """
    if total < 1:
        raise ValueError("total must be at least 1")

    with open(outfile, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["username", "password"])

        if include_admin:
            # Reserve one slot for admin
            writer.writerow(["adminX", "123X"])
            remaining = total - 1
        else:
            remaining = total

        # If remaining is 0, we only wanted the admin user
        for i in range(1, remaining + 1):
            username = f"user{i}"
            password = f"pass{i}"
            writer.writerow([username, password])

    return outfile

def parse_args(argv):
    p = argparse.ArgumentParser(description="Generate a CSV file of users.")
    p.add_argument("--total", type=int, default=1000,
                   help="Total number of users to generate (including admin). Default: 1000")
    p.add_argument("--outfile", type=str, default="users.csv",
                   help="Output CSV filename. Default: users.csv")
    p.add_argument("--no-admin", action="store_true",
                   help="Do not include the admin,123 row.")
    return p.parse_args(argv)

def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    try:
        out = generate_users_csv(total=args.total,
                                 outfile=args.outfile,
                                 include_admin=not args.no_admin)
        print(f"Generated {args.total} users and wrote to: {out}")
    except Exception as e:
        print("Error:", e, file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
