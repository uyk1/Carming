import {createSlice, PayloadAction} from '@reduxjs/toolkit';
import {Place} from '../../types';

type SliceState = {
  placeList: Place[];
  currentIdx: number;
};

const journeySlice = createSlice({
  name: 'journeySlice',
  initialState: {
    placeList: [],
    currentIdx: 0,
  } as SliceState,
  reducers: {
    // Place List
    setJourneyPlaceList: (state, action: PayloadAction<Place[]>) => {
      const newPlaceList = action.payload;
      state.placeList = [...newPlaceList];
    },

    // Current Index
    setCurrentIdx: (state, action: PayloadAction<number>) => {
      state.currentIdx = action.payload;
    },
  },
});

export default journeySlice;
export const {setJourneyPlaceList, setCurrentIdx} = journeySlice.actions;
