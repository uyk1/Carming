import {Place} from './Place';

export type PopularCourseResponse = {
  id: number;
  ratingCount: number;
  ratingSum: number;
  placeNames: string[];
};

export type PopularCourseDetailResponse = {
  name: string;
  ratingCount: number;
  ratingSum: number;
  regions: string[];
  places: Place[];
};

export type SelectedPopularPlaceResponse = {
  id: number;
  name: string;
  image: string;
  region: string;
  address: string;
  ratingSum: number;
  ratingCount: number;
  tag: {
    tagName: string;
    tagCount: number;
  }[];
};
