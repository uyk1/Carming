import axios from 'axios';
import {Coordinate} from '../types';

const callNaverDirectionApi = async (coordinates: Coordinate[]) => {
  const start = `${coordinates[0].longitude},${coordinates[0].latitude}`;
  const end = `${coordinates[coordinates.length - 1].longitude},${
    coordinates[coordinates.length - 1].latitude
  }`;
  const waypoints = coordinates
    .slice(1, coordinates.length - 1)
    .reduce((prev, curr) => prev + `${curr.longitude},${curr.latitude}|`, '')
    .slice(0, -1);
  return axios.get(
    `https://naveropenapi.apigw.ntruss.com/map-direction-15/v1/driving?start=${start}&goal=${end}&waypoints=${waypoints}`,
    {
      headers: {
        'X-NCP-APIGW-API-KEY-ID': '9zcu207bzs',
        'X-NCP-APIGW-API-KEY': 'Sa1cNjNkp70zrMBtjxannzf1rcI94Wh905saO9LI',
      },
    },
  );
};

export {callNaverDirectionApi};
