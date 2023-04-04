import {createApi} from '@reduxjs/toolkit/query/react';
import {REST_API_URL} from '@env';
import {Category, Place} from '../types';
import customFetchBaseQuery from './customFetchBaseQuery';

export interface PlaceSearch {
  regions: string[];
  category: Category;
  size: number;
  page: number;
}

export const placeApi = createApi({
  reducerPath: 'placeApi',
  baseQuery: customFetchBaseQuery({baseUrl: REST_API_URL + '/places'}),
  // baseQuery: customFetchBaseQuery({
  //   baseUrl: 'http://10.0.2.2:8080/api' + '/places',
  // }),
  tagTypes: ['Places'],
  endpoints: builder => ({
    getPlaces: builder.query<Place[], PlaceSearch>({
      query: filter => ({
        url: '/',
        params: filter,
      }),
      serializeQueryArgs: endpointName => {
        console.log(
          'serializeQueryArgs endpointName ::: ',
          endpointName.queryArgs.category,
        );
        return endpointName.queryArgs.category;
      },
      merge: (currentCache, newItems) => {
        currentCache.push(...newItems);
      },
      forceRefetch: ({currentArg, previousArg}) => {
        console.log(
          'forceRefetch currentArg, previousArg, return ::: ',
          currentArg,
          previousArg,
          currentArg?.category !== previousArg?.category,
        );
        return currentArg?.page !== previousArg?.page;
      },
      providesTags: ['Places'],
    }),
  }),
});

export const {useGetPlacesQuery} = placeApi;
