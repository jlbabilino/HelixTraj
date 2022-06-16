from app.app import app
import click

@click.command()
@click.argument('input')
@click.argument('output')
@click.option('--path', default='path', help='path to use in document')
def cli(input, output, path):
    '''
        A tool for generating time-optimal trajectories for FRC robots using CasADi.
        Accepts waypoints and robot characteristics and outputs a list of sample
        points that a trajectory follower can use.
    '''
    app(input, output, path)

if __name__ == '__main__':
    cli()